// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlTreeSolver.hpp"

#include <algorithm> // std::find
#include <sstream>
#include <string>

#include <yarp/conf/version.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/segment.hpp>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treeiksolverpos_online.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>

#include "KinematicRepresentation.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_KINEMATICS = "none.ini";
constexpr auto DEFAULT_LAMBDA = 0.01;
constexpr auto DEFAULT_EPS = 1e-6;
constexpr auto DEFAULT_MAXITER = 1000;
constexpr auto DEFAULT_V_TRANSL_MAX = 1.0; // meters/s
constexpr auto DEFAULT_V_ROT_MAX = 50.0; // degrees/s
constexpr auto DEFAULT_IK_SOLVER = "nrjl";

// ------------------- DeviceDriver Related ------------------------------------

namespace
{
    bool getMatrixFromProperties(const yarp::os::Searchable & options, const std::string & tag, Eigen::MatrixXd & mat)
    {
        const auto * bot = options.find(tag).asList();

        if (!bot)
        {
            return false;
        }

        int i = 0;
        int j = 0;

        for (int cnt = 0; cnt < bot->size() && cnt < mat.rows() * mat.cols(); cnt++)
        {
            mat(i, j) = bot->get(cnt).asFloat64();

            if (++j >= mat.cols())
            {
                i++;
                j = 0;
            }
        }

        std::stringstream ss;
        ss << "Matrix " << tag << ":\n" << mat;
        yCInfo(KDLS) << ss.str();

        return true;
    }

    bool retrieveJointLimits(const yarp::os::Searchable & options, KDL::JntArray & qMin, KDL::JntArray & qMax, KDL::JntArray & qMaxVel)
    {
        int nrOfJoints = qMin.rows();

        if (!options.check("mins") || !options.check("maxs") || !options.check("maxvels"))
        {
            yCError(KDLS) << "Missing 'mins' and/or 'maxs' and/or 'maxvels' option(s)";
            return false;
        }

        auto * maxs = options.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();
        auto * mins = options.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();
        auto * maxvels = options.findGroup("maxvels", "joint velocity (meters/second or degrees/second)").get(1).asList();

        if (!maxs || maxs->isNull() || !mins || mins->isNull() || !maxvels || maxvels->isNull())
        {
            yCError(KDLS) << "Empty 'mins' and/or 'maxs' and/or 'maxvels' option(s)";
            return false;
        }

        if (maxs->size() != nrOfJoints || mins->size() != nrOfJoints || maxvels->size() != nrOfJoints)
        {
            yCError(KDLS, "chain.getNrOfJoints (%d) != maxs.size() or mins.size() or maxvels.size() (%zu, %zu, %zu)",
                    nrOfJoints, maxs->size(), mins->size(), maxvels->size());
            return false;
        }

        yCDebug(KDLS) << "qMax:" << maxs->toString();
        yCDebug(KDLS) << "qMin:" << mins->toString();
        yCDebug(KDLS) << "qMaxVel:" << maxvels->toString();

        for (int motor = 0; motor < nrOfJoints; motor++)
        {
            qMax(motor) = KinRepresentation::degToRad(maxs->get(motor).asFloat64());
            qMin(motor) = KinRepresentation::degToRad(mins->get(motor).asFloat64());
            qMaxVel(motor) = KinRepresentation::degToRad(maxvels->get(motor).asFloat64());

            if (qMin(motor) == qMax(motor))
            {
                yCWarning(KDLS, "qMin[%1$d] == qMax[%1$d] (%2$f)", motor, qMin(motor));
            }
            else if (qMin(motor) > qMax(motor))
            {
                yCError(KDLS, "qMin[%1$d] > qMax[%1$d] (%2$f > %3$f)", motor, qMin(motor), qMax(motor));
                return false;
            }
        }

        return true;
    }
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::open(yarp::os::Searchable & config)
{
#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(KDLS) << "config:" << config.toString();
#endif

    //-- kinematics
    std::string kinematics = config.check("kinematics", yarp::os::Value(DEFAULT_KINEMATICS),
        "path to file with description of robot kinematics").asString();

    yCInfo(KDLS) << "kinematics:" << kinematics;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("kinematics");
    std::string kinematicsFullPath = rf.findFileByName(kinematics);

    yarp::os::Property fullConfig;
    fullConfig.fromConfigFile(kinematicsFullPath.c_str());
    fullConfig.fromString(config.toString(), false);
    fullConfig.setMonitor(config.getMonitor(), "KdlTreeSolver");

    yCDebug(KDLS) << "fullConfig:" << fullConfig.toString();

    yarp::os::Bottle chains = fullConfig.findGroup("chain", "kinematic chains").tail();

    if (chains.isNull() || chains.size() == 0)
    {
        yCError(KDLS) << "Missing or empty \"chain\" section collection";
        return false;
    }

    //-- gravity (default)
    yarp::os::Value defaultGravityValue;
    yarp::os::Bottle * defaultGravityBottle = defaultGravityValue.asList();
    defaultGravityBottle->addFloat64(0.0);
    defaultGravityBottle->addFloat64(0.0);
    defaultGravityBottle->addFloat64(-9.81);

    //-- gravity
    yarp::os::Value gravityValue = fullConfig.check("gravity", defaultGravityValue, "gravity vector (SI units)");
    yarp::os::Bottle * gravityBottle = gravityValue.asList();
    KDL::Vector gravity(gravityBottle->get(0).asFloat64(), gravityBottle->get(1).asFloat64(), gravityBottle->get(2).asFloat64());
    yCInfo(KDLS) << "gravity:" << gravityBottle->toString();

    for (int i = 0; i < chains.size(); i++)
    {
        KDL::Chain chain;

        auto chainName = chains.get(i).asString();
        yCInfo(KDLS) << "chain:" << chainName;

        const yarp::os::Bottle & chainConfig = fullConfig.findGroup(chainName);
        yCDebug(KDLS) << "config:" << chainConfig.toString();

        //-- hook
        auto hook = chainConfig.check("hook", yarp::os::Value("root"), "segment to hook to at its end").asString();
        yCInfo(KDLS) << "hook:" << hook;

        //-- numlinks
        int numLinks = chainConfig.check("numLinks", yarp::os::Value(0), "chain number of segments").asInt32();
        yCInfo(KDLS) << "numLinks:" << numLinks;

        //-- H0
        Eigen::MatrixXd H0 = Eigen::MatrixXd::Identity(4, 4);

        if (!getMatrixFromProperties(chainConfig, "H0", H0))
        {
            yCWarning(KDLS) << "Failed to parse H0, using default identity matrix";
        }

        std::string H0name = chainName + "_H0";
        KDL::Vector kdlVec0(H0(0, 3), H0(1, 3), H0(2, 3));
        KDL::Rotation kdlRot0(H0(0, 0), H0(0, 1), H0(0, 2), H0(1, 0), H0(1, 1), H0(1, 2), H0(2, 0), H0(2, 1), H0(2, 2));
        chain.addSegment(KDL::Segment(H0name, KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRot0, kdlVec0)));

        //-- links
        for (int linkIndex = 0; linkIndex < numLinks; linkIndex++)
        {
            std::string link = "link_" + std::to_string(linkIndex);
            yarp::os::Bottle & bLink = chainConfig.findGroup(link);

            if (!bLink.isNull())
            {
                std::string segmentName = chainName + "_" + link;

                //-- Kinematic
                double linkOffset = bLink.check("offset", yarp::os::Value(0.0), "DH joint angle (degrees)").asFloat64();
                double linkD = bLink.check("D", yarp::os::Value(0.0), "DH link offset (meters)").asFloat64();
                double linkA = bLink.check("A", yarp::os::Value(0.0), "DH link length (meters)").asFloat64();
                double linkAlpha = bLink.check("alpha", yarp::os::Value(0.0), "DH link twist (degrees)").asFloat64();

                KDL::Joint axis(KDL::Joint::RotZ);
                KDL::Frame H = KDL::Frame::DH(linkA, KinRepresentation::degToRad(linkAlpha), linkD, KinRepresentation::degToRad(linkOffset));

                //-- Dynamic
                if (bLink.check("mass") && bLink.check("cog") && bLink.check("inertia"))
                {
                    double linkMass = bLink.check("mass", yarp::os::Value(0.0), "link mass (SI units)").asFloat64();
                    yarp::os::Bottle linkCog = bLink.findGroup("cog", "vector of link's center of gravity (SI units)").tail();
                    yarp::os::Bottle linkInertia = bLink.findGroup("inertia", "vector of link's inertia (SI units)").tail();

                    KDL::Vector cog(linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64());
                    KDL::RotationalInertia inertia(linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64());

                    chain.addSegment(KDL::Segment(segmentName, axis, H, KDL::RigidBodyInertia(linkMass, cog, inertia)));

                    yCInfo(KDLS, "Added: %s (offset %f) (D %f) (A %f) (alpha %f) (mass %f) (cog %f %f %f) (inertia %f %f %f)",
                           link.c_str(), linkOffset, linkD, linkA, linkAlpha, linkMass,
                           linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64(),
                           linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64());
                }
                else
                {
                    chain.addSegment(KDL::Segment(segmentName, axis, H));
                    yCInfo(KDLS, "Added: %s (offset %f) (D %f) (A %f) (alpha %f)", link.c_str(), linkOffset, linkD, linkA, linkAlpha);
                }
            }
            else
            {
                std::string xyzLink = "xyzLink_" + std::to_string(linkIndex);
                yCWarning(KDLS, "Not found: \"%s\", looking for \"%s\" instead.", link.c_str(), xyzLink.c_str());

                std::string segmentName = chainName + "_" + xyzLink;
                yarp::os::Bottle & bXyzLink = chainConfig.findGroup(xyzLink);

                if (bXyzLink.isNull())
                {
                    yCError(KDLS, "Not found: \"%s\" either.", xyzLink.c_str());
                    return false;
                }

                double linkX = bXyzLink.check("x", yarp::os::Value(0.0), "X coordinate of next frame (meters)").asFloat64();
                double linkY = bXyzLink.check("y", yarp::os::Value(0.0), "Y coordinate of next frame (meters)").asFloat64();
                double linkZ = bXyzLink.check("z", yarp::os::Value(0.0), "Z coordinate of next frame (meters)").asFloat64();

                std::string linkTypes = "joint type (Rot[XYZ]|InvRot[XYZ]|Trans[XYZ]|InvTrans[XYZ]), e.g. 'RotZ'";
                std::string linkType = bXyzLink.check("Type", yarp::os::Value("NULL"), linkTypes.c_str()).asString();

                KDL::Frame H(KDL::Vector(linkX, linkY, linkZ));

                if (linkType == "RotX") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::RotX), H));
                } else if (linkType == "RotY") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::RotY), H));
                } else if (linkType == "RotZ") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::RotZ), H));
                } else if (linkType == "InvRotX") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::RotX, -1.0), H));
                } else if (linkType == "InvRotY") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::RotY, -1.0), H));
                } else if (linkType == "InvRotZ") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::RotZ, -1.0), H));
                } else if (linkType == "TransX") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::TransX), H));
                } else if (linkType == "TransY") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::TransY), H));
                } else if (linkType == "TransZ") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::TransZ), H));
                } else if (linkType == "InvTransX") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::TransX, -1.0), H));
                } else if (linkType == "InvTransY") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::TransY, -1.0), H));
                } else if (linkType == "InvTransZ") {
                    chain.addSegment(KDL::Segment(segmentName, KDL::Joint(KDL::Joint::TransZ, -1.0), H));
                } else {
                    yCWarning(KDLS, "Link joint type \"%s\" unrecognized!", linkType.c_str());
                }

                yCInfo(KDLS, "Added: %s (Type %s) (x %f) (y %f) (z %f)", xyzLink.c_str(), linkType.c_str(), linkX, linkY, linkZ);
            }
        }

        //-- HN
        Eigen::MatrixXd HN = Eigen::MatrixXd::Identity(4, 4);

        if (!getMatrixFromProperties(chainConfig, "HN", HN))
        {
            yCWarning(KDLS) << "Failed to parse HN, using default identity matrix";
        }

        KDL::Vector kdlVecN(HN(0, 3), HN(1, 3), HN(2, 3));
        KDL::Rotation kdlRotN(HN(0, 0), HN(0, 1), HN(0, 2), HN(1, 0), HN(1, 1), HN(1, 2), HN(2, 0), HN(2, 1), HN(2, 2));
        chain.addSegment(KDL::Segment(chainName, KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRotN, kdlVecN)));

        yCInfo(KDLS) << "Chain number of segments:" << chain.getNrOfSegments();
        yCInfo(KDLS) << "Chain number of joints:" << chain.getNrOfJoints();

        if (!tree.addChain(chain, hook))
        {
            yCError(KDLS) << "Unable to add chain to tree, hook name" << hook << "not found";
            return false;
        }

        if (chainConfig.check("endpoint", yarp::os::Value(false), "whether this chain is an endpoint (TCP)").asBool())
        {
            endpoints.push_back(chainName);

            if (chainConfig.check("mergeWith", "other chain's TCP to merge this TCP with"))
            {
                auto mergeWith = chainConfig.find("mergeWith").asString();

                if (std::find(endpoints.cbegin(), endpoints.cend(), mergeWith) == endpoints.cend())
                {
                    yCError(KDLS) << "Unable to find TCP" << mergeWith << "to be merged with" << chainName;
                    return false;
                }

                mergedEndpoints.emplace(chainName, mergeWith);
                yCInfo(KDLS) << "TCP" << chainName << "merged with" << mergeWith;
            }
        }
    }

    yCInfo(KDLS) << "Tree number of segments:" << tree.getNrOfSegments();
    yCInfo(KDLS) << "Tree number of joints:" << tree.getNrOfJoints();
    yCInfo(KDLS) << "Tree endpoints:" << endpoints;

    fkSolverPos = new KDL::TreeFkSolverPos_recursive(tree);
    ikSolverVel = new KDL::TreeIkSolverVel_wdls(tree, endpoints);
    idSolver = new KDL::TreeIdSolver_RNE(tree, gravity);

    {
        auto * temp = dynamic_cast<KDL::TreeIkSolverVel_wdls *>(ikSolverVel);

        auto lambda = fullConfig.check("lambda", yarp::os::Value(DEFAULT_LAMBDA), "lambda parameter for diff IK").asFloat64();
        temp->setLambda(lambda); // disclaimer: never set this to zero (which is the default)

        Eigen::MatrixXd wJS = Eigen::MatrixXd::Identity(tree.getNrOfJoints(), tree.getNrOfJoints());

        if (!getMatrixFromProperties(fullConfig, "weightJS", wJS))
        {
            yCWarning(KDLS) << "Failed to parse weightJS, using default identity matrix";
        }

        temp->setWeightJS(wJS);

        Eigen::MatrixXd wTS = Eigen::MatrixXd::Identity(6 * endpoints.size(), 6 * endpoints.size());

        if (!getMatrixFromProperties(fullConfig, "weightTS", wTS))
        {
            yCWarning(KDLS) << "Failed to parse weightTS, using default identity matrix";
        }

        temp->setWeightTS(wTS);
    }

    KDL::JntArray qMax(tree.getNrOfJoints());
    KDL::JntArray qMin(tree.getNrOfJoints());
    KDL::JntArray qMaxVels(tree.getNrOfJoints());

    //-- Joint limits.
    if (!retrieveJointLimits(fullConfig, qMin, qMax, qMaxVels))
    {
        yCError(KDLS) << "Unable to retrieve joint limits";
        return false;
    }

    //-- IK solver algorithm.
    std::string ik = fullConfig.check("ik", yarp::os::Value(DEFAULT_IK_SOLVER), "IK solver algorithm (nrjl, online)").asString();

    if (ik == "nrjl")
    {
        //-- Precision and max iterations.
        double eps = fullConfig.check("eps", yarp::os::Value(DEFAULT_EPS), "IK solver precision (meters)").asFloat64();
        int maxIter = fullConfig.check("maxIter", yarp::os::Value(DEFAULT_MAXITER), "maximum number of iterations").asInt32();

        ikSolverPos = new KDL::TreeIkSolverPos_NR_JL(tree, endpoints, qMin, qMax, *fkSolverPos, *ikSolverVel, maxIter, eps);
    }
    else if (ik == "online")
    {
        //-- Max cartesian speed.
        double vTranslMax = fullConfig.check("vTranslMax", yarp::os::Value(DEFAULT_V_TRANSL_MAX), "maximum translation speed (meters/second)").asFloat64();
        double vRotMax = fullConfig.check("vRotMax", yarp::os::Value(DEFAULT_V_ROT_MAX), "maximum rotation speed (degrees/second)").asFloat64();

        vRotMax = KinRepresentation::degToRad(vRotMax);
        ikSolverPos = new KDL::TreeIkSolverPos_Online(tree.getNrOfJoints(), endpoints, qMin, qMax, qMaxVels, vTranslMax, vRotMax, *fkSolverPos, *ikSolverVel);
    }
    else
    {
        yCError(KDLS) << "Unsupported IK solver algorithm:" << ik.c_str();
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::close()
{
    delete fkSolverPos;
    fkSolverPos = nullptr;

    delete ikSolverPos;
    ikSolverPos = nullptr;

    delete ikSolverVel;
    ikSolverVel = nullptr;

    delete idSolver;
    idSolver = nullptr;

    return true;
}

// -----------------------------------------------------------------------------

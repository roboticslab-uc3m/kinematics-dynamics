// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlTreeSolver.hpp"

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>

#include <yarp/sig/Matrix.h>

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

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

namespace
{
    bool getMatrixFromProperties(const yarp::os::Searchable & options, const std::string & tag, yarp::sig::Matrix & mat)
    {
        yarp::os::Bottle * bot = options.find(tag).asList();

        if (!bot)
        {
            yWarning() << "Unable to find tag" << tag.c_str();
            return false;
        }

        int i = 0;
        int j = 0;

        mat.zero();

        for (int cnt = 0; cnt < bot->size() && cnt < mat.rows() * mat.cols(); cnt++)
        {
            mat(i, j) = bot->get(cnt).asFloat64();

            if (++j >= mat.cols())
            {
                i++;
                j = 0;
            }
        }

        return true;
    }

    bool retrieveJointLimits(const yarp::os::Searchable & options, KDL::JntArray & qMin, KDL::JntArray & qMax, KDL::JntArray & qMaxVel)
    {
        int nrOfJoints = qMin.rows();

        if (!options.check("mins") || !options.check("maxs") || !options.check("maxvels"))
        {
            yError() << "Missing 'mins' and/or 'maxs' and/or 'maxvels' option(s)";
            return false;
        }

        auto * maxs = options.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();
        auto * mins = options.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();
        auto * maxvels = options.findGroup("maxvels", "joint velocity (meters/second or degrees/second)").get(1).asList();

        if (!maxs || maxs->isNull() || !mins || mins->isNull() || !maxvels || maxvels->isNull())
        {
            yError() << "Empty 'mins' and/or 'maxs' and/or 'maxvels' option(s)";
            return false;
        }

        if (maxs->size() < nrOfJoints || mins->size() < nrOfJoints || maxvels->size() < nrOfJoints)
        {
            yError("chain.getNrOfJoints (%d) > maxs.size() or mins.size() or maxvels.size() (%zu, %zu, %zu)",
                nrOfJoints, maxs->size(), mins->size(), maxvels->size());
            return false;
        }

        yDebug() << "qMax:" << maxs->toString();
        yDebug() << "qMin:" << mins->toString();
        yDebug() << "qMaxVel:" << maxvels->toString();

        for (int motor = 0; motor < nrOfJoints; motor++)
        {
            qMax(motor) = KinRepresentation::degToRad(maxs->get(motor).asFloat64());
            qMin(motor) = KinRepresentation::degToRad(mins->get(motor).asFloat64());
            qMaxVel(motor) = KinRepresentation::degToRad(maxvels->get(motor).asFloat64());

            if (qMin(motor) == qMax(motor))
            {
                yWarning("qMin[%1$d] == qMax[%1$d] (%2$f)", motor, qMin(motor));
            }
            else if (qMin(motor) > qMax(motor))
            {
                yError("qMin[%1$d] > qMax[%1$d] (%2$f > %3$f)", motor, qMin(motor), qMax(motor));
                return false;
            }
        }

        return true;
    }
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::open(yarp::os::Searchable & config)
{
    yDebug() << "config:" << config.toString();

    //-- kinematics
    std::string kinematics = config.check("kinematics", yarp::os::Value(DEFAULT_KINEMATICS),
        "path to file with description of robot kinematics").asString();

    yInfo() << "kinematics:" << kinematics;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("kinematics");
    std::string kinematicsFullPath = rf.findFileByName(kinematics);

    yarp::os::Property fullConfig;
    fullConfig.fromConfigFile(kinematicsFullPath.c_str());
    fullConfig.fromString(config.toString(), false);
    fullConfig.setMonitor(config.getMonitor(), "KdlTreeSolver");

    yDebug() << "fullConfig:" << fullConfig.toString();

    yarp::os::Bottle chains = fullConfig.findGroup("chain", "kinematic chains").tail();

    if (chains.isNull() || chains.size() == 0)
    {
        yError() << "Missing or empty \"chain\" section collection";
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
    yInfo() << "gravity:" << gravityBottle->toString();

    //-- H0 (default)
    yarp::sig::Matrix defaultYmH0(4, 4);
    defaultYmH0.eye();

    //-- HN (default)
    yarp::sig::Matrix defaultYmHN(4, 4);
    defaultYmHN.eye();

    for (int i = 0; i < chains.size(); i++)
    {
        KDL::Chain chain;

        auto chainName = chains.get(i).asString();
        yInfo() << "chain:" << chainName;

        const yarp::os::Bottle & chainConfig = fullConfig.findGroup(chainName);
        yDebug() << "config:" << chainConfig.toString();

        //-- hook
        auto hook = chainConfig.check("hook", yarp::os::Value("root"), "segment to hook to at its end").asString();
        yInfo() << "hook:" << hook;

        //-- numlinks
        int numLinks = chainConfig.check("numLinks", yarp::os::Value(0), "chain number of segments").asInt32();
        yInfo() << "numLinks:" << numLinks;

        //-- H0
        yarp::sig::Matrix ymH0(4, 4);
        std::string ymH0_str = "H0";

        if (!getMatrixFromProperties(chainConfig, ymH0_str, ymH0))
        {
            ymH0 = defaultYmH0;
        }

        std::string H0name = chainName + "_H0";
        KDL::Vector kdlVec0(ymH0(0, 3), ymH0(1, 3), ymH0(2, 3));
        KDL::Rotation kdlRot0(ymH0(0, 0), ymH0(0, 1), ymH0(0, 2), ymH0(1, 0), ymH0(1, 1), ymH0(1, 2), ymH0(2, 0), ymH0(2, 1), ymH0(2, 2));
        chain.addSegment(KDL::Segment(H0name, KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRot0, kdlVec0)));
        yInfo() << "H0:\n" << ymH0.toString();

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

                    yInfo("Added: %s (offset %f) (D %f) (A %f) (alpha %f) (mass %f) (cog %f %f %f) (inertia %f %f %f)",
                          link.c_str(), linkOffset, linkD, linkA, linkAlpha, linkMass,
                          linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64(),
                          linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64());
                }
                else
                {
                    chain.addSegment(KDL::Segment(segmentName, axis, H));
                    yInfo("Added: %s (offset %f) (D %f) (A %f) (alpha %f)", link.c_str(), linkOffset, linkD, linkA, linkAlpha);
                }
            }
            else
            {
                std::string xyzLink = "xyzLink_" + std::to_string(linkIndex);
                yWarning("Not found: \"%s\", looking for \"%s\" instead.", link.c_str(), xyzLink.c_str());

                std::string segmentName = chainName + "_" + xyzLink;
                yarp::os::Bottle & bXyzLink = chainConfig.findGroup(xyzLink);

                if (bXyzLink.isNull())
                {
                    yError("Not found: \"%s\" either.", xyzLink.c_str());
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
                    yWarning("Link joint type \"%s\" unrecognized!", linkType.c_str());
                }

                yInfo("Added: %s (Type %s) (x %f) (y %f) (z %f)", xyzLink.c_str(), linkType.c_str(), linkX, linkY, linkZ);
            }
        }

        //-- HN
        yarp::sig::Matrix ymHN(4, 4);
        std::string ymHN_str = "HN";

        if (!getMatrixFromProperties(chainConfig, ymHN_str, ymHN))
        {
            ymHN = defaultYmHN;
        }

        KDL::Vector kdlVecN(ymHN(0, 3), ymHN(1, 3), ymHN(2, 3));
        KDL::Rotation kdlRotN(ymHN(0, 0), ymHN(0, 1), ymHN(0, 2), ymHN(1, 0), ymHN(1, 1), ymHN(1, 2), ymHN(2, 0), ymHN(2, 1), ymHN(2, 2));
        chain.addSegment(KDL::Segment(chainName, KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRotN, kdlVecN)));
        yInfo() << "HN:\n" << ymHN.toString();

        yInfo() << "Chain number of segments:" << chain.getNrOfSegments();
        yInfo() << "Chain number of joints:" << chain.getNrOfJoints();

        if (!tree.addChain(chain, hook))
        {
            yError() << "Unable to add chain to tree, hook name" << hook << "not found";
            return false;
        }

        bool isEndpoint = chainConfig.check("endpoint", yarp::os::Value(false), "whether this chain is an endpoint").asBool();

        if (isEndpoint)
        {
            endpoints.push_back(chainName);
        }
    }

    yInfo() << "Tree number of segments:" << tree.getNrOfSegments();
    yInfo() << "Tree number of joints:" << tree.getNrOfJoints();
    yInfo() << "Tree endpoints:" << endpoints;

    fkSolverPos = new KDL::TreeFkSolverPos_recursive(tree);
    ikSolverVel = new KDL::TreeIkSolverVel_wdls(tree, endpoints);
    idSolver = new KDL::TreeIdSolver_RNE(tree, gravity);

    {
        auto * temp = dynamic_cast<KDL::TreeIkSolverVel_wdls *>(ikSolverVel);

        auto lambda = fullConfig.check("lambda", yarp::os::Value(DEFAULT_LAMBDA), "lambda parameter for diff IK").asFloat64();
        temp->setLambda(lambda); // disclaimer: never set this to zero (which is the default)

        yarp::sig::Matrix wJS(tree.getNrOfJoints(), tree.getNrOfJoints());

        if (!getMatrixFromProperties(fullConfig, "weightJS", wJS))
        {
            wJS.eye();
        }

        Eigen::MatrixXd _wJS(wJS.rows(), wJS.cols());

        for (auto i = 0; i < wJS.rows(); i++)
        {
            for (auto j = 0; j < wJS.cols(); j++)
            {
                _wJS(i, j) = wJS(i, j);
            }
        }

        temp->setWeightJS(_wJS);

        yarp::sig::Matrix wTS(6 * endpoints.size(), 6 * endpoints.size());

        if (!getMatrixFromProperties(fullConfig, "weightTS", wTS))
        {
            wTS.eye();
        }

        Eigen::MatrixXd _wTS(wTS.rows(), wTS.cols());

        for (auto i = 0; i < wTS.rows(); i++)
        {
            for (auto j = 0; j < wTS.cols(); j++)
            {
                _wTS(i, j) = wTS(i, j);
            }
        }

        temp->setWeightTS(_wTS);
    }

    KDL::JntArray qMax(tree.getNrOfJoints());
    KDL::JntArray qMin(tree.getNrOfJoints());
    KDL::JntArray qMaxVels(tree.getNrOfJoints());

    //-- Joint limits.
    if (!retrieveJointLimits(fullConfig, qMin, qMax, qMaxVels))
    {
        yError() << "Unable to retrieve joint limits";
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
        yError() << "Unsupported IK solver algorithm:" << ik.c_str();
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

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

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
#include <kdl/utilities/utility.h> // KDL::deg2rad

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include "ConfigurationSelector.hpp"

#include "ChainIkSolverPos_ST.hpp"
#include "ChainIkSolverPos_ID.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_KINEMATICS = "none.ini";
constexpr auto DEFAULT_NUM_LINKS = 1;
constexpr auto DEFAULT_EPS_POS = 1e-5;
constexpr auto DEFAULT_EPS_VEL = 1e-5;
constexpr auto DEFAULT_MAXITER_POS = 1000;
constexpr auto DEFAULT_MAXITER_VEL = 150;
constexpr auto DEFAULT_IK_POS_SOLVER = "st";
constexpr auto DEFAULT_IK_VEL_SOLVER = "pinv";
constexpr auto DEFAULT_LMA_WEIGHTS = "1 1 1 0.1 0.1 0.1";
constexpr auto DEFAULT_LAMBDA = 0.01;
constexpr auto DEFAULT_STRATEGY = "leastOverallAngularDisplacement";

// ------------------- DeviceDriver Related ------------------------------------

namespace
{
    bool getMatrixFromProperties(const yarp::os::Searchable & options, const std::string & tag, Eigen::MatrixXd & mat)
    {
        const auto * bH = options.find(tag).asList();

        if (!bH)
        {
            return false;
        }

        int i = 0;
        int j = 0;

        for (int cnt = 0; cnt < bH->size() && cnt < mat.rows() * mat.cols(); cnt++)
        {
            mat(i, j) = bH->get(cnt).asFloat64();

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

    bool parseLmaFromBottle(const yarp::os::Bottle & b, Eigen::Matrix<double, 6, 1> & L)
    {
        if (b.size() != 6)
        {
            yCWarning(KDLS, "Wrong bottle size (expected: %d, was: %zu)", 6, b.size());
            return false;
        }

        for (int i = 0; i < b.size(); i++)
        {
            L(i) = b.get(i).asFloat64();
        }

        return true;
    }

    bool retrieveJointLimits(const yarp::os::Searchable & options, KDL::JntArray & qMin, KDL::JntArray & qMax)
    {
        int nrOfJoints = qMin.rows();

        if (!options.check("mins") || !options.check("maxs"))
        {
            yCError(KDLS) << "Missing 'mins' and/or 'maxs' option(s)";
            return false;
        }

        const auto * maxs = options.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();
        const auto * mins = options.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();

        if (!maxs || maxs->isNull() || !mins || mins->isNull())
        {
            yCError(KDLS) << "Empty 'mins' and/or 'maxs' option(s)";
            return false;
        }

        if (maxs->size() != nrOfJoints || mins->size() != nrOfJoints)
        {
            yCError(KDLS, "chain.getNrOfJoints (%d) != maxs.size() or mins.size() (%zu, %zu)", nrOfJoints, maxs->size(), mins->size());
            return false;
        }

        yCDebug(KDLS) << "qMax:" << maxs->toString();
        yCDebug(KDLS) << "qMin:" << mins->toString();

        for (int motor = 0; motor < nrOfJoints; motor++)
        {
            qMax(motor) = maxs->get(motor).asFloat64() * KDL::deg2rad;
            qMin(motor) = mins->get(motor).asFloat64() * KDL::deg2rad;

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

bool KdlSolver::open(yarp::os::Searchable & config)
{
    isQuiet = config.check("quiet", "disable logging");

    //-- kinematics
    std::string kinematics = config.check("kinematics", yarp::os::Value(DEFAULT_KINEMATICS),
        "path to file with description of robot kinematics").asString();

    yCInfo(KDLS) << "kinematics:" << kinematics;

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("kinematics");
    std::string kinematicsFullPath = rf.findFileByName(kinematics);

    yarp::os::Property fullConfig;
    fullConfig.fromConfigFile(kinematicsFullPath.c_str());
    fullConfig.fromString(config.toString(), false);
    fullConfig.setMonitor(config.getMonitor(), "KdlSolver");

    yCDebug(KDLS) << "fullConfig:" << fullConfig.toString();

    //-- numlinks
    int numLinks = fullConfig.check("numLinks", yarp::os::Value(DEFAULT_NUM_LINKS), "chain number of segments").asInt32();
    yCInfo(KDLS) << "numLinks:" << numLinks;

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

    //-- H0
    Eigen::MatrixXd H0 = Eigen::MatrixXd::Identity(4, 4);

    if (!getMatrixFromProperties(fullConfig, "H0", H0))
    {
        yCWarning(KDLS) << "Failed to parse H0, using default identity matrix";
    }

    KDL::Vector kdlVec0(H0(0, 3), H0(1, 3), H0(2, 3));
    KDL::Rotation kdlRot0(H0(0, 0), H0(0, 1), H0(0, 2), H0(1, 0), H0(1, 1), H0(1, 2), H0(2, 0), H0(2, 1), H0(2, 2));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRot0, kdlVec0)));

    //-- links
    for (int linkIndex = 0; linkIndex < numLinks; linkIndex++)
    {
        std::string link = "link_" + std::to_string(linkIndex);
        yarp::os::Bottle & bLink = fullConfig.findGroup(link);

        if (!bLink.isNull())
        {
            //-- Kinematic
            double linkOffset = bLink.check("offset", yarp::os::Value(0.0), "DH joint angle (degrees)").asFloat64();
            double linkD = bLink.check("D", yarp::os::Value(0.0), "DH link offset (meters)").asFloat64();
            double linkA = bLink.check("A", yarp::os::Value(0.0), "DH link length (meters)").asFloat64();
            double linkAlpha = bLink.check("alpha", yarp::os::Value(0.0), "DH link twist (degrees)").asFloat64();

            KDL::Joint axis(KDL::Joint::RotZ);
            KDL::Frame H = KDL::Frame::DH(linkA, linkAlpha * KDL::deg2rad, linkD, linkOffset * KDL::deg2rad);

            //-- Dynamic
            if (bLink.check("mass") && bLink.check("cog") && bLink.check("inertia"))
            {
                double linkMass = bLink.check("mass", yarp::os::Value(0.0), "link mass (SI units)").asFloat64();
                yarp::os::Bottle linkCog = bLink.findGroup("cog", "vector of link's center of gravity (SI units)").tail();
                yarp::os::Bottle linkInertia = bLink.findGroup("inertia", "vector of link's inertia (SI units)").tail();

                KDL::Vector cog(linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64());
                KDL::RotationalInertia inertia(linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64());

                chain.addSegment(KDL::Segment(axis, H, KDL::RigidBodyInertia(linkMass, cog, inertia)));

                yCInfo(KDLS, "Added: %s (offset %f) (D %f) (A %f) (alpha %f) (mass %f) (cog %f %f %f) (inertia %f %f %f)",
                       link.c_str(), linkOffset, linkD, linkA, linkAlpha, linkMass,
                       linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64(),
                       linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64());
            }
            else
            {
                chain.addSegment(KDL::Segment(axis, H));
                yCInfo(KDLS, "Added: %s (offset %f) (D %f) (A %f) (alpha %f)", link.c_str(), linkOffset, linkD, linkA, linkAlpha);
            }
        }
        else
        {
            std::string xyzLink = "xyzLink_" + std::to_string(linkIndex);
            yCWarning(KDLS, "Not found: \"%s\", looking for \"%s\" instead.", link.c_str(), xyzLink.c_str());

            yarp::os::Bottle & bXyzLink = fullConfig.findGroup(xyzLink);

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
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), H));
            } else if (linkType == "RotY") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), H));
            } else if (linkType == "RotZ") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), H));
            } else if (linkType == "InvRotX") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX, -1.0), H));
            } else if (linkType == "InvRotY") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY, -1.0), H));
            } else if (linkType == "InvRotZ") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, -1.0), H));
            } else if (linkType == "TransX") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX), H));
            } else if (linkType == "TransY") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY), H));
            } else if (linkType == "TransZ") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ), H));
            } else if (linkType == "InvTransX") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX, -1.0), H));
            } else if (linkType == "InvTransY") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY, -1.0), H));
            } else if (linkType == "InvTransZ") {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ, -1.0), H));
            } else {
                yCWarning(KDLS, "Link joint type \"%s\" unrecognized!", linkType.c_str());
            }

            yCInfo(KDLS, "Added: %s (Type %s) (x %f) (y %f) (z %f)", xyzLink.c_str(), linkType.c_str(), linkX, linkY, linkZ);
        }
    }

    //-- HN
    Eigen::MatrixXd HN = Eigen::MatrixXd::Identity(4, 4);

    if (!getMatrixFromProperties(fullConfig, "HN", HN))
    {
        yCWarning(KDLS) << "Failed to parse HN, using default identity matrix";
    }

    KDL::Vector kdlVecN(HN(0, 3), HN(1, 3), HN(2, 3));
    KDL::Rotation kdlRotN(HN(0, 0), HN(0, 1), HN(0, 2), HN(1, 0), HN(1, 1), HN(1, 2), HN(2, 0), HN(2, 1), HN(2, 2));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRotN, kdlVecN)));

    yCInfo(KDLS) << "Chain number of segments:" << chain.getNrOfSegments();
    yCInfo(KDLS) << "Chain number of joints:" << chain.getNrOfJoints();

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    idSolver = new KDL::ChainIdSolver_RNE(chain, gravity);

    //-- IK vel solver algorithm.
    auto ikVel = fullConfig.check("ikVel", yarp::os::Value(DEFAULT_IK_VEL_SOLVER), "IK velocity solver algorithm (pinv, wdls)").asString();

    if (ikVel == "pinv")
    {
        double eps = fullConfig.check("epsVel", yarp::os::Value(DEFAULT_EPS_VEL), "IK velocity solver precision (meters)").asFloat64();
        double maxIter = fullConfig.check("maxIterVel", yarp::os::Value(DEFAULT_MAXITER_VEL), "IK velocity solver max iterations").asInt32();

        ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, eps, maxIter);
    }
    else if (ikVel == "wdls")
    {
        double lambda = fullConfig.check("lambda", yarp::os::Value(DEFAULT_LAMBDA), "lambda parameter for diff IK").asFloat64();
        double eps = fullConfig.check("epsVel", yarp::os::Value(DEFAULT_EPS_VEL), "IK velocity solver precision (meters)").asFloat64();
        double maxIter = fullConfig.check("maxIterVel", yarp::os::Value(DEFAULT_MAXITER_VEL), "IK velocity solver max iterations").asInt32();

        ikSolverVel = new KDL::ChainIkSolverVel_wdls(chain, eps, maxIter);

        auto * temp = dynamic_cast<KDL::ChainIkSolverVel_wdls *>(ikSolverVel);
        temp->setLambda(lambda);

        Eigen::MatrixXd wJS = Eigen::MatrixXd::Identity(chain.getNrOfJoints(), chain.getNrOfJoints());

        if (!getMatrixFromProperties(fullConfig, "weightJS", wJS))
        {
            yCWarning(KDLS) << "Failed to parse weightJS, using default identity matrix";
        }

        temp->setWeightJS(wJS);

        Eigen::MatrixXd wTS = Eigen::MatrixXd::Identity(6, 6);

        if (!getMatrixFromProperties(fullConfig, "weightTS", wTS))
        {
            yCWarning(KDLS) << "Failed to parse weightTS, using default identity matrix";
        }

        temp->setWeightTS(wTS);
    }
    else
    {
        yCError(KDLS) << "Unsupported IK velocity solver algorithm:" << ikVel.c_str();
        return false;
    }

    //-- IK pos solver algorithm.
    auto ik = fullConfig.check("ik", yarp::os::Value(DEFAULT_IK_POS_SOLVER), "IK solver algorithm (lma, nrjl, st, id)"); // back-compat
    auto ikPos = fullConfig.check("ikPos", ik, "IK position solver algorithm (lma, nrjl, st, id)").asString();

    if (ikPos == "lma")
    {
        std::string weightsStr = fullConfig.check("weights", yarp::os::Value(DEFAULT_LMA_WEIGHTS), "LMA algorithm weights (bottle of 6 doubles)").asString();
        yarp::os::Bottle weights(weightsStr);
        Eigen::Matrix<double, 6, 1> L;

        if (!parseLmaFromBottle(weights, L))
        {
            yCError(KDLS) << "Unable to parse LMA weights";
            return false;
        }

        double eps = fullConfig.check("epsPos", yarp::os::Value(DEFAULT_EPS_POS), "IK position solver precision (meters)").asFloat64();
        double maxIter = fullConfig.check("maxIterPos", yarp::os::Value(DEFAULT_MAXITER_POS), "IK position solver max iterations").asInt32();

        ikSolverPos = new KDL::ChainIkSolverPos_LMA(chain, L, eps, maxIter);
    }
    else if (ikPos == "nrjl")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            yCError(KDLS) << "Unable to retrieve joint limits";
            return false;
        }

        double eps = fullConfig.check("epsPos", yarp::os::Value(DEFAULT_EPS_POS), "IK position solver precision (meters)").asFloat64();
        double maxIter = fullConfig.check("maxIterPos", yarp::os::Value(DEFAULT_MAXITER_POS), "IK position solver max iterations").asInt32();

        ikSolverPos = new KDL::ChainIkSolverPos_NR_JL(chain, qMin, qMax, *fkSolverPos, *ikSolverVel, maxIter, eps);
    }
    else if (ikPos == "st")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        //-- Joint limits.
        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            yCError(KDLS) << "Unable to retrieve joint limits";
            return false;
        }

        //-- IK configuration selection strategy.
        std::string strategy = fullConfig.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

        if (strategy == "leastOverallAngularDisplacement")
        {
            ConfigurationSelectorLeastOverallAngularDisplacementFactory factory(qMin, qMax);
            ikSolverPos = ChainIkSolverPos_ST::create(chain, factory);
        }
        else if (strategy == "humanoidGait")
        {
            ConfigurationSelectorHumanoidGaitFactory factory(qMin, qMax);
            ikSolverPos = ChainIkSolverPos_ST::create(chain, factory);
        }
        else
        {
            yCError(KDLS) << "Unsupported IK strategy:" << strategy;
            return false;
        }

        if (!ikSolverPos)
        {
            yCError(KDLS) << "Unable to solve IK";
            return false;
        }
    }
    else if (ikPos == "id")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        //-- Joint limits.
        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            yCError(KDLS) << "Unable to retrieve joint limits";
            return false;
        }

        ikSolverPos = new ChainIkSolverPos_ID(chain, qMin, qMax, *fkSolverPos);
    }
    else
    {
        yCError(KDLS) << "Unsupported IK position solver algorithm:" << ikPos.c_str();
        return false;
    }

    originalChain = chain;

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::close()
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

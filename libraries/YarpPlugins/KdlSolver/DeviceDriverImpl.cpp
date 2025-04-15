// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

#include <cmath>

#include <string>

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

// ------------------- DeviceDriver Related ------------------------------------

namespace
{
    Eigen::MatrixXd getMatrixFromVector(const std::vector<double> & v)
    {
        const int size = std::sqrt(static_cast<double>(v.size()));
        Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(size, size);

        for (int r = 0, c = 0, i = 0; i < v.size() && i < mat.rows() * mat.cols(); i++)
        {
            mat(r, c) = v[i];

            if (++c >= mat.cols())
            {
                r++;
                c = 0;
            }
        }

        return mat;
    }

    KDL::JntArray retrieveJointLimits(const std::vector<double> in)
    {
        // https://forum.kde.org/viewtopic.php%3Ff=74&t=94839.html#p331301
        KDL::JntArray ret;
        ret.data = Eigen::VectorXd::Map(in.data(), in.size());
        return ret;
    }
}

// -----------------------------------------------------------------------------

bool KdlSolver::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(KDLS) << "Failed to parse parameters";
        return false;
    }

    yarp::os::Property kinematicsConfig;

    if (const auto & kinematicsGroup = config.findGroup("KINEMATICS"); !kinematicsGroup.isNull())
    {
        yCDebug(KDLS) << "Parsing kinematics group";
        kinematicsConfig.fromString(kinematicsGroup.toString().c_str());
    }
    else if (!m_kinematics.empty())
    {
        yCDebug(KDLS) << "Parsing kinematics file" << m_kinematics;

        yarp::os::ResourceFinder rf;
        rf.setDefaultContext("kinematics");

        std::string kinematicsFullPath = rf.findFileByName(m_kinematics);

        if (!kinematicsConfig.fromConfigFile(kinematicsFullPath.c_str()))
        {
            yCError(KDLS) << "Could not configure kinematics from" << kinematicsFullPath;
            return false;
        }
    }
    else
    {
        yCError(KDLS) << "No kinematics group nor file provided";
        return false;
    }

    yCDebug(KDLS) << "kinematicsConfig:" << kinematicsConfig.toString();

    if (!makeChain(kinematicsConfig, chain))
    {
        yCError(KDLS) << "Failed to create kinematic chain";
        return false;
    }

    KDL::Vector gravity(m_gravity[0], m_gravity[1], m_gravity[2]);

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    idSolver = new KDL::ChainIdSolver_RNE(chain, gravity);

    if (m_ikVel == "pinv")
    {
        ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, m_epsVel, m_maxIterVel);
    }
    else if (m_ikVel == "wdls")
    {
        ikSolverVel = new KDL::ChainIkSolverVel_wdls(chain, m_epsVel, m_maxIterVel);

        auto * temp = dynamic_cast<KDL::ChainIkSolverVel_wdls *>(ikSolverVel);
        temp->setLambda(m_lambda);

        if (m_weightsJS.size() != std::pow(chain.getNrOfJoints(), 2))
        {
            yCWarning(KDLS) << "Illegal size of weightJS";
        }

        temp->setWeightJS(getMatrixFromVector(m_weightsJS));

        if (m_weightsTS.size() != std::pow(6, 2))
        {
            yCWarning(KDLS) << "Illegal size of weightTS";
        }

        temp->setWeightTS(getMatrixFromVector(m_weightsTS));
    }
    else
    {
        yCError(KDLS) << "Unsupported IK velocity solver algorithm:" << m_ikVel;
        return false;
    }

    if (m_ikPos == "lma")
    {
        Eigen::Matrix<double, 6, 1> L(m_weightsLMA.data());
        ikSolverPos = new KDL::ChainIkSolverPos_LMA(chain, L, m_epsPos, m_maxIterPos);
    }
    else if (m_ikPos == "nrjl")
    {
        KDL::JntArray qMax = retrieveJointLimits(m_maxs);
        KDL::JntArray qMin = retrieveJointLimits(m_mins);

        ikSolverPos = new KDL::ChainIkSolverPos_NR_JL(chain, qMin, qMax, *fkSolverPos, *ikSolverVel, m_maxIterPos, m_epsPos);
    }
    else if (m_ikPos == "st")
    {
        KDL::JntArray qMax = retrieveJointLimits(m_maxs);
        KDL::JntArray qMin = retrieveJointLimits(m_mins);

        if (m_invKinStrategy == "leastOverallAngularDisplacement")
        {
            ConfigurationSelectorLeastOverallAngularDisplacementFactory factory(qMin, qMax);
            ikSolverPos = ChainIkSolverPos_ST::create(chain, factory);
        }
        else if (m_invKinStrategy == "humanoidGait")
        {
            ConfigurationSelectorHumanoidGaitFactory factory(qMin, qMax);
            ikSolverPos = ChainIkSolverPos_ST::create(chain, factory);
        }
        else
        {
            yCError(KDLS) << "Unsupported IK strategy:" << m_invKinStrategy;
            return false;
        }

        if (!ikSolverPos)
        {
            yCError(KDLS) << "Unable to solve IK";
            return false;
        }
    }
    else if (m_ikPos == "id")
    {
        KDL::JntArray qMax = retrieveJointLimits(m_maxs);
        KDL::JntArray qMin = retrieveJointLimits(m_mins);

        ikSolverPos = new ChainIkSolverPos_ID(chain, qMin, qMax, *fkSolverPos);
    }
    else
    {
        yCError(KDLS) << "Unsupported IK position solver algorithm:" << m_ikPos;
        return false;
    }

    originalChain = chain;

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::makeChain(const yarp::os::Searchable & kinConfig, KDL::Chain & chain)
{
    int numLinks = kinConfig.check("numLinks", yarp::os::Value(0), "chain number of segments").asInt32();
    yCInfo(KDLS) << "numLinks:" << numLinks;

    if (const auto * H0 = kinConfig.find("H0").asList(); H0 != nullptr)
    {
        if (H0->size() != 16)
        {
            yCError(KDLS) << "Invalid or missing H0 matrix";
            return false;
        }

        KDL::Vector kdlVec0(H0->get(3).asFloat64(), H0->get(7).asFloat64(), H0->get(11).asFloat64());

        KDL::Rotation kdlRot0(
            H0->get(0).asFloat64(), H0->get(1).asFloat64(), H0->get(2).asFloat64(),
            H0->get(4).asFloat64(), H0->get(5).asFloat64(), H0->get(6).asFloat64(),
            H0->get(8).asFloat64(), H0->get(9).asFloat64(), H0->get(10).asFloat64());

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRot0, kdlVec0)));
    }

    for (int linkIndex = 0; linkIndex < numLinks; linkIndex++)
    {
        std::string link = "link_" + std::to_string(linkIndex);
        yarp::os::Bottle & bLink = kinConfig.findGroup(link);

        if (!bLink.isNull())
        {
            double linkOffset = bLink.check("offset", yarp::os::Value(0.0), "DH joint angle (degrees)").asFloat64();
            double linkD = bLink.check("D", yarp::os::Value(0.0), "DH link offset (meters)").asFloat64();
            double linkA = bLink.check("A", yarp::os::Value(0.0), "DH link length (meters)").asFloat64();
            double linkAlpha = bLink.check("alpha", yarp::os::Value(0.0), "DH link twist (degrees)").asFloat64();

            KDL::Joint axis(KDL::Joint::RotZ);
            KDL::Frame H = KDL::Frame::DH(linkA, linkAlpha * KDL::deg2rad, linkD, linkOffset * KDL::deg2rad);

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

            yarp::os::Bottle & bXyzLink = kinConfig.findGroup(xyzLink);

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

            KDL::Frame H({linkX, linkY, linkZ});

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

    if (const auto * HN = kinConfig.find("HN").asList(); HN != nullptr)
    {
        if (HN->size() != 16)
        {
            yCError(KDLS) << "Invalid HN matrix";
            return false;
        }

        KDL::Vector kdlVecN(HN->get(3).asFloat64(), HN->get(7).asFloat64(), HN->get(11).asFloat64());

        KDL::Rotation kdlRotN(
            HN->get(0).asFloat64(), HN->get(1).asFloat64(), HN->get(2).asFloat64(),
            HN->get(4).asFloat64(), HN->get(5).asFloat64(), HN->get(6).asFloat64(),
            HN->get(8).asFloat64(), HN->get(9).asFloat64(), HN->get(10).asFloat64());

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRotN, kdlVecN)));
    }

    yCInfo(KDLS) << "Chain number of segments:" << chain.getNrOfSegments();
    yCInfo(KDLS) << "Chain number of joints:" << chain.getNrOfJoints();

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

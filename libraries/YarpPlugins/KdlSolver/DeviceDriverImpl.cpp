// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

#include <string>

#include <yarp/os/Bottle.h>
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

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <Eigen/Core> // Eigen::Matrix

#include <ColorDebug.h>

#include "KinematicRepresentation.hpp"

#include "ChainIkSolverPos_ST.hpp"
#include "ChainIkSolverPos_ID.hpp"

// ------------------- DeviceDriver Related ------------------------------------

namespace
{
    bool getMatrixFromProperties(const yarp::os::Searchable & options, const std::string & tag, yarp::sig::Matrix & H)
    {
        yarp::os::Bottle * bH = options.find(tag).asList();

        if (!bH)
        {
            CD_WARNING("Unable to find tag %s.\n", tag.c_str());
            return false;
        }

        int i = 0;
        int j = 0;

        H.zero();

        for (int cnt = 0; cnt < bH->size() && cnt < H.rows() * H.cols(); cnt++)
        {
            H(i, j) = bH->get(cnt).asDouble();

            if (++j >= H.cols())
            {
                i++;
                j = 0;
            }
        }

        return true;
    }

    bool parseLmaFromBottle(const yarp::os::Bottle & b, Eigen::Matrix<double, 6, 1> & L)
    {
        if (b.size() != 6)
        {
            CD_WARNING("Wrong bottle size (expected: %d, was: %d).\n", 6, b.size());
            return false;
        }

        for (int i = 0; i < b.size(); i++)
        {
            L(i) = b.get(i).asDouble();
        }

        return true;
    }

    bool retrieveJointLimits(const yarp::os::Searchable & options, KDL::JntArray & qMin, KDL::JntArray & qMax)
    {
        int nrOfJoints = qMin.rows();

        if (!options.check("mins") || !options.check("maxs"))
        {
            CD_ERROR("Missing 'mins' and/or 'maxs' option(s).\n");
            return false;
        }

        yarp::os::Bottle * maxs = options.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();
        yarp::os::Bottle * mins = options.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();

        if (maxs == YARP_NULLPTR || mins == YARP_NULLPTR)
        {
            CD_ERROR("Empty 'mins' and/or 'maxs' option(s)\n");
            return false;
        }

        if (maxs->size() < nrOfJoints || mins->size() < nrOfJoints)
        {
            CD_ERROR("chain.getNrOfJoints (%d) > maxs.size() or mins.size() (%d, %d)\n", nrOfJoints, maxs->size(), mins->size());
            return false;
        }

        for (int motor = 0; motor < nrOfJoints; motor++)
        {
            qMax(motor) = maxs->get(motor).asDouble();
            qMin(motor) = mins->get(motor).asDouble();

            if (qMin(motor) == qMax(motor))
            {
                CD_WARNING("qMin[%1$d] == qMax[%1$d] (%2$f)\n", motor, qMin(motor));
            }
            else if (qMin(motor) > qMax(motor))
            {
                CD_ERROR("qMin[%1$d] > qMax[%1$d] (%2$f > %3$f)\n", motor, qMin(motor), qMax(motor));
                return false;
            }
        }

        return true;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::open(yarp::os::Searchable& config)
{
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    //-- kinematics
    std::string kinematics = config.check("kinematics",yarp::os::Value(DEFAULT_KINEMATICS),"path to file with description of robot kinematics").asString();
    CD_INFO("kinematics: %s [%s]\n", kinematics.c_str(),DEFAULT_KINEMATICS);
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("kinematics");
    std::string kinematicsFullPath = rf.findFileByName(kinematics);

    yarp::os::Property fullConfig;
    fullConfig.fromConfigFile(kinematicsFullPath.c_str());
    fullConfig.fromString(config.toString(),false);  //-- Can override kinematics file contents.
    fullConfig.setMonitor(config.getMonitor(), "KdlSolver");

    CD_DEBUG("fullConfig: %s.\n", fullConfig.toString().c_str());

    //-- numlinks
    int numLinks = fullConfig.check("numLinks",yarp::os::Value(DEFAULT_NUM_LINKS),"chain number of segments").asInt();
    CD_INFO("numLinks: %d [%d]\n",numLinks,DEFAULT_NUM_LINKS);

    //-- gravity
    yarp::os::Value defaultGravityValue;
    yarp::os::Bottle *defaultGravityBottle = defaultGravityValue.asList();
    defaultGravityBottle->addDouble(0);
    defaultGravityBottle->addDouble(0);
    defaultGravityBottle->addDouble(-9.81);

    yarp::os::Value gravityValue = fullConfig.check("gravity", defaultGravityValue, "gravity vector (SI units)");
    yarp::os::Bottle *gravityBottle = gravityValue.asList();
    KDL::Vector gravity(gravityBottle->get(0).asDouble(),gravityBottle->get(1).asDouble(),gravityBottle->get(2).asDouble());
    CD_INFO("gravity: %s [%s]\n",gravityBottle->toString().c_str(),defaultGravityBottle->toString().c_str());

    //-- H0
    yarp::sig::Matrix defaultYmH0(4,4);
    defaultYmH0.eye();

    yarp::sig::Matrix ymH0(4,4);
    std::string ymH0_str("H0");
    if( ! getMatrixFromProperties(fullConfig, ymH0_str, ymH0))
    {
        ymH0 = defaultYmH0;
    }

    KDL::Vector kdlVec0(ymH0(0,3),ymH0(1,3),ymH0(2,3));
    KDL::Rotation kdlRot0( ymH0(0,0),ymH0(0,1),ymH0(0,2),ymH0(1,0),ymH0(1,1),ymH0(1,2),ymH0(2,0),ymH0(2,1),ymH0(2,2));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRot0,kdlVec0)));  //-- H0 = Frame(kdlRot0,kdlVec0);
    CD_INFO("H0:\n%s\n[%s]\n",ymH0.toString().c_str(),defaultYmH0.toString().c_str());

    //-- links
    for(int linkIndex=0;linkIndex<numLinks;linkIndex++)
    {
        std::string link("link_");
        std::ostringstream s;
        s << linkIndex;
        link += s.str();
        yarp::os::Bottle &bLink = fullConfig.findGroup(link);
        if( ! bLink.isNull() ) {
            //-- Kinematic
            double linkOffset = bLink.check("offset",yarp::os::Value(0.0), "DH joint angle (degrees)").asDouble();
            double linkD = bLink.check("D",yarp::os::Value(0.0), "DH link offset (meters)").asDouble();
            double linkA = bLink.check("A",yarp::os::Value(0.0), "DH link length (meters)").asDouble();
            double linkAlpha = bLink.check("alpha",yarp::os::Value(0.0), "DH link twist (degrees)").asDouble();
            //-- Dynamic
            if( bLink.check("mass") && bLink.check("cog") && bLink.check("inertia")) {
                double linkMass = bLink.check("mass",yarp::os::Value(0.0), "link mass (SI units)").asDouble();
                yarp::os::Bottle linkCog = bLink.findGroup("cog", "vector of link's center of gravity (SI units)").tail();
                yarp::os::Bottle linkInertia = bLink.findGroup("inertia", "vector of link's inertia (SI units)").tail();
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(linkA,KinRepresentation::degToRad(linkAlpha),linkD,KinRepresentation::degToRad(linkOffset)),
                                              KDL::RigidBodyInertia(linkMass,KDL::Vector(linkCog.get(0).asDouble(),linkCog.get(1).asDouble(),linkCog.get(2).asDouble()),
                                                                    KDL::RotationalInertia(linkInertia.get(0).asDouble(),linkInertia.get(1).asDouble(),linkInertia.get(2).asDouble(),0,0,0))));
                CD_INFO("Added: %s (offset %f) (D %f) (A %f) (alpha %f) (mass %f) (cog %f %f %f) (inertia %f %f %f)\n",
                           link.c_str(), linkOffset,linkD,linkA,linkAlpha,linkMass,
                           linkCog.get(0).asDouble(),linkCog.get(1).asDouble(),linkCog.get(2).asDouble(),
                           linkInertia.get(0).asDouble(),linkInertia.get(1).asDouble(),linkInertia.get(2).asDouble());
            }
            else //-- No mass -> skip dynamics
            {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(linkA,KinRepresentation::degToRad(linkAlpha),linkD,KinRepresentation::degToRad(linkOffset))));
                CD_INFO("Added: %s (offset %f) (D %f) (A %f) (alpha %f)\n",link.c_str(), linkOffset,linkD,linkA,linkAlpha);
            }
            continue;
        }

        std::string xyzLink("xyzLink_");
        std::ostringstream xyzS;
        xyzS << linkIndex;
        xyzLink += xyzS.str();
        CD_WARNING("Not found: \"%s\", looking for \"%s\" instead.\n", link.c_str(), xyzLink.c_str());
        yarp::os::Bottle &bXyzLink = fullConfig.findGroup(xyzLink);
        if( bXyzLink.isNull() ) {
            CD_ERROR("Not found: \"%s\" either.\n", xyzLink.c_str());
            return false;
        }
        double linkX = bXyzLink.check("x",yarp::os::Value(0.0), "X coordinate of next frame (meters)").asDouble();
        double linkY = bXyzLink.check("y",yarp::os::Value(0.0), "Y coordinate of next frame (meters)").asDouble();
        double linkZ = bXyzLink.check("z",yarp::os::Value(0.0), "Z coordinate of next frame (meters)").asDouble();

        std::string linkTypes = "joint type (Rot[XYZ]|InvRot[XYZ]|Trans[XYZ]|InvTrans[XYZ]), e.g. 'RotZ'";
        std::string linkType = bXyzLink.check("Type",yarp::os::Value("NULL"), linkTypes.c_str()).asString();
        if(linkType == "RotX") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "RotY") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "RotZ") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "InvRotX") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "InvRotY") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "InvRotZ") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "TransX") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "TransY") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "TransZ") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "InvTransX") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "InvTransY") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else if(linkType == "InvTransZ") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
        } else {
            CD_WARNING("Link joint type \"%s\" unrecognized!\n",linkType.c_str());
        }

        CD_SUCCESS("Added: %s (Type %s) (x %f) (y %f) (z %f)\n",xyzLink.c_str(),linkType.c_str(),linkX,linkY,linkZ);
    }

    //-- HN
    yarp::sig::Matrix defaultYmHN(4,4);
    defaultYmHN.eye();

    yarp::sig::Matrix ymHN(4,4);
    std::string ymHN_str("HN");
    if( ! getMatrixFromProperties(fullConfig, ymHN_str, ymHN))
    {
        ymHN = defaultYmHN;
    }

    KDL::Vector kdlVecN(ymHN(0,3),ymHN(1,3),ymHN(2,3));
    KDL::Rotation kdlRotN( ymHN(0,0),ymHN(0,1),ymHN(0,2),ymHN(1,0),ymHN(1,1),ymHN(1,2),ymHN(2,0),ymHN(2,1),ymHN(2,2));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRotN,kdlVecN)));
    CD_INFO("HN:\n%s\n[%s]\n",ymHN.toString().c_str(),defaultYmHN.toString().c_str());

    CD_INFO("Chain number of segments (post- H0 and HN): %d\n",chain.getNrOfSegments());
    CD_INFO("Chain number of joints (post- H0 and HN): %d\n",chain.getNrOfJoints());

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain);
    idSolver = new KDL::ChainIdSolver_RNE(chain, gravity);

    //-- IK solver algorithm.
    std::string ik = fullConfig.check("ik", yarp::os::Value(DEFAULT_IK_SOLVER), "IK solver algorithm (lma, nrjl, st, id)").asString();

    if (ik == "lma")
    {
        std::string weightsStr = fullConfig.check("weights", yarp::os::Value(DEFAULT_LMA_WEIGHTS), "LMA algorithm weights (bottle of 6 doubles)").asString();
        yarp::os::Bottle weights(weightsStr);
        Eigen::Matrix<double, 6, 1> L;

        if (!parseLmaFromBottle(weights, L))
        {
            CD_ERROR("Unable to parse LMA weights.\n");
            return false;
        }

        ikSolverPos = new KDL::ChainIkSolverPos_LMA(chain, L);
    }
    else if (ik == "nrjl")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        //-- Joint limits.
        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            CD_ERROR("Unable to retrieve joint limits.\n");
            return false;
        }

        //-- Precision and max iterations.
        double eps = fullConfig.check("eps", yarp::os::Value(DEFAULT_EPS), "IK solver precision (meters)").asDouble();
        double maxIter = fullConfig.check("maxIter", yarp::os::Value(DEFAULT_MAXITER), "maximum number of iterations").asInt();

        ikSolverPos = new KDL::ChainIkSolverPos_NR_JL(chain, qMin, qMax, *fkSolverPos, *ikSolverVel, maxIter, eps);
    }
    else if (ik == "st")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        //-- Joint limits.
        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            CD_ERROR("Unable to retrieve joint limits.\n");
            return false;
        }

        //-- IK configuration selection strategy.
        std::string strategy = fullConfig.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

        if (strategy == "leastOverallAngularDisplacement")
        {
            ConfigurationSelectorLeastOverallAngularDisplacementFactory factory(qMin, qMax);
            ikSolverPos = ChainIkSolverPos_ST::create(chain, factory);
        }
        else
        {
            CD_ERROR("Unsupported IK strategy: %s.\n", strategy.c_str());
            return false;
        }

        if (ikSolverPos == NULL)
        {
            CD_ERROR("Unable to solve IK.\n");
            return false;
        }
    }
    else if (ik == "id")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        //-- Joint limits.
        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            CD_ERROR("Unable to retrieve joint limits.\n");
            return false;
        }

        ikSolverPos = new ChainIkSolverPos_ID(chain, qMin, qMax, *fkSolverPos);
    }
    else
    {
        CD_ERROR("Unsupported IK solver algorithm: %s.\n", ik.c_str());
        return false;
    }

    originalChain = chain;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::close()
{
    delete fkSolverPos;
    delete ikSolverPos;
    delete ikSolverVel;
    delete idSolver;

    return true;
}

// -----------------------------------------------------------------------------

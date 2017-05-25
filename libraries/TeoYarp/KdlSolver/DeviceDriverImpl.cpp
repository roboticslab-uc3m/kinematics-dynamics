// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::KdlSolver::open(yarp::os::Searchable& config)
{

    CD_DEBUG("config: %s.\n", config.toString().c_str());

    //-- kinematics
    std::string kinematics = config.check("kinematics",yarp::os::Value(DEFAULT_KINEMATICS),"limb kinematic description").asString();
    CD_INFO("kinematics: %s [%s]\n", kinematics.c_str(),DEFAULT_KINEMATICS);
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("kinematics");
    std::string kinematicsFullPath = rf.findFileByName(kinematics);

    yarp::os::Property fullConfig;
    fullConfig.fromConfigFile(kinematicsFullPath.c_str());
    fullConfig.fromString(config.toString(),false);  //-- Can override kinematics file contents.

    CD_DEBUG("fullConfig: %s.\n", fullConfig.toString().c_str());

    //-- numlinks
    int numLinks = fullConfig.check("numLinks",yarp::os::Value(DEFAULT_NUM_LINKS),"chain number of segments").asInt();
    CD_INFO("numLinks: %d [%d]\n",numLinks,DEFAULT_NUM_LINKS);

    //-- angleRepr
    angleRepr = fullConfig.check("angleRepr",yarp::os::Value(DEFAULT_ANGLE_REPR),"axisAngle, eulerYZ, eulerZYZ or RPY").asString();
    CD_INFO("angleRepr: %s [%s]\n",angleRepr.c_str(),DEFAULT_ANGLE_REPR);

    if( ! ( (angleRepr == "axisAngle")
            || (angleRepr == "eulerYZ")
            || (angleRepr == "eulerZYZ")
            || (angleRepr == "RPY") ) )
    {
        CD_ERROR("Did not recognize angleRepr: %s.\n",angleRepr.c_str());
        return false;
    }

    //-- gravity
    yarp::os::Bottle defaultGravityBottle;
    defaultGravityBottle.addDouble(0);
    defaultGravityBottle.addDouble(0);
    defaultGravityBottle.addDouble(-9.81);

    yarp::os::Bottle gravityBottle;
    if( fullConfig.check("gravity") )
    {
        gravityBottle = fullConfig.findGroup("gravity").tail();
    }
    else
    {
        gravityBottle = defaultGravityBottle;
    }
    gravity = KDL::Vector(gravityBottle.get(0).asDouble(),gravityBottle.get(1).asDouble(),gravityBottle.get(2).asDouble());
    CD_INFO("gravity: %s [%s]\n",gravityBottle.toString().c_str(),defaultGravityBottle.toString().c_str());

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
            double linkOffset = bLink.check("offset",yarp::os::Value(0.0)).asDouble();
            double linkD = bLink.check("D",yarp::os::Value(0.0)).asDouble();
            double linkA = bLink.check("A",yarp::os::Value(0.0)).asDouble();
            double linkAlpha = bLink.check("alpha",yarp::os::Value(0.0)).asDouble();
            //-- Dynamic
            if( bLink.check("mass") && bLink.check("cog") && bLink.check("inertia")) {
                double linkMass = bLink.check("mass",yarp::os::Value(0.0)).asDouble();
                yarp::os::Bottle linkCog = bLink.findGroup("cog").tail();
                yarp::os::Bottle linkInertia = bLink.findGroup("inertia").tail();
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(linkA,toRad(linkAlpha),linkD,toRad(linkOffset)),
                                              KDL::RigidBodyInertia(linkMass,KDL::Vector(linkCog.get(0).asDouble(),linkCog.get(1).asDouble(),linkCog.get(2).asDouble()),
                                                                    KDL::RotationalInertia(linkInertia.get(0).asDouble(),linkInertia.get(1).asDouble(),linkInertia.get(2).asDouble(),0,0,0))));
                CD_INFO("Added: %s (offset %f) (D %f) (A %f) (alpha %f) (mass %f) (cog %f %f %f) (inertia %f %f %f)\n",
                           link.c_str(), linkOffset,linkD,linkA,linkAlpha,linkMass,
                           linkCog.get(0).asDouble(),linkCog.get(1).asDouble(),linkCog.get(2).asDouble(),
                           linkInertia.get(0).asDouble(),linkInertia.get(1).asDouble(),linkInertia.get(2).asDouble());
            }
            else //-- No mass -> skip dynamics
            {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(linkA,toRad(linkAlpha),linkD,toRad(linkOffset))));
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
        double linkX = bXyzLink.check("x",yarp::os::Value(0.0)).asDouble();
        double linkY = bXyzLink.check("y",yarp::os::Value(0.0)).asDouble();
        double linkZ = bXyzLink.check("z",yarp::os::Value(0.0)).asDouble();

        std::string linkType = bXyzLink.check("Type",yarp::os::Value("NULL")).asString();
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

    //--
    CD_INFO("Chain number of segments (post- H0 and HN): %d\n",chain.getNrOfSegments());
    CD_INFO("Chain number of joints (post- H0 and HN): %d\n",chain.getNrOfJoints());

    qMax.resize(chain.getNrOfJoints());
    qMin.resize(chain.getNrOfJoints());
    //-- Limits [ -pi , pi ].
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
    {
        qMax(motor) = M_PI;
        qMin(motor) = -M_PI;
    }

    originalChain = chain;  // We have: Chain& operator = (const Chain& arg);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::close() {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::getMatrixFromProperties(yarp::os::Searchable &options, std::string &tag, yarp::sig::Matrix &H) {

    yarp::os::Bottle *bH=options.find(tag).asList();
    if (!bH) return false;

    int i=0;
    int j=0;
    H.zero();
    for (int cnt=0; (cnt<bH->size()) && (cnt<H.rows()*H.cols()); cnt++) {
        H(i,j)=bH->get(cnt).asDouble();
        if (++j>=H.cols()) {
            i++;
            j=0;
        }
    }
    return true;
}

// -----------------------------------------------------------------------------

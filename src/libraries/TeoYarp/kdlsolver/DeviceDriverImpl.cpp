// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::KdlSolver::open(yarp::os::Searchable& config) {

    numLinks = config.check("numLinks",yarp::os::Value(DEFAULT_NUM_LINKS),"chain number of segments").asInt();
    angleRepr = config.check("angleRepr",yarp::os::Value(DEFAULT_ANGLE_REPR),"axisAngle, eulerYZ, eulerZYZ or RPY").asString();
    yarp::os::Bottle gravityBottle = config.findGroup("g").tail();

    //Vector(0.0,-10,0.0)

    if(angleRepr == "axisAngle") {
        targetO.resize(4);
    } else if(angleRepr == "eulerYZ") {
        targetO.resize(2);
    } else if(angleRepr == "eulerZYZ") {
        targetO.resize(3);
    } else if(angleRepr == "RPY") {
        targetO.resize(3);
    } else {
        CD_WARNING("Did not recognize angleRepr: %s.\n",angleRepr.c_str());
    }

    for(int linkIndex=0;linkIndex<numLinks;linkIndex++) {

        std::string link("link_");
        std::ostringstream s;
        s << linkIndex;
        link += s.str();
        yarp::os::Bottle &bLink = config.findGroup(link);
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
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH(linkA,toRad(linkAlpha),linkD,toRad(linkOffset)),
                                              KDL::RigidBodyInertia(linkMass,KDL::Vector(linkCog.get(0).asDouble(),linkCog.get(1).asDouble(),linkCog.get(2).asDouble()),
                                                                    KDL::RotationalInertia(linkInertia.get(0).asDouble(),linkInertia.get(1).asDouble(),linkInertia.get(2).asDouble(),0,0,0))));
                CD_SUCCESS("Added: %s (offset %f) (D %f) (A %f) (alpha %f) (mass %f) (cog %f %f %f) (inertia %f %f %f)\n",
                           link.c_str(), linkOffset,linkD,linkA,linkAlpha,linkMass,
                           linkCog.get(0).asDouble(),linkCog.get(1).asDouble(),linkCog.get(2).asDouble(),
                           linkInertia.get(0).asDouble(),linkInertia.get(1).asDouble(),linkInertia.get(2).asDouble());
            }
            else //-- No mass -> skip dynamics
            {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH(linkA,toRad(linkAlpha),linkD,toRad(linkOffset))));
                CD_SUCCESS("Added: %s (offset %f) (D %f) (A %f) (alpha %f)\n",link.c_str(), linkOffset,linkD,linkA,linkAlpha);
            }
            continue;
        }

        std::string xyzLink("xyzLink_");
        std::ostringstream xyzS;
        xyzS << linkIndex;
        xyzLink += xyzS.str();
        CD_WARNING("Not found: \"%s\", looking for \"%s\" instead.\n", link.c_str(), xyzLink.c_str());
        yarp::os::Bottle &bXyzLink = config.findGroup(xyzLink);
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
            isPrismatic.push_back(0);
        } else if(linkType == "RotY") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "RotZ") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "InvRotX") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "InvRotY") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "InvRotZ") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "TransX") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "TransY") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "TransZ") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "InvTransX") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "InvTransY") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "InvTransZ") {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ,-1.0),KDL::Frame(KDL::Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else {
            CD_WARNING("Link joint type \"%s\" unrecognized!\n",linkType.c_str());
        }

        CD_SUCCESS("Added: %s (Type %s) (x %f) (y %f) (z %f)\n",xyzLink.c_str(),linkType.c_str(),linkX,linkY,linkZ);
    }

    printf("KdlSolver chain number of segments: %d\n",chain.getNrOfSegments());

    _orient = new KDL::RotationalInterpolation_SingleAxis();
    _eqradius = 1; //0.000001;
    _aggregate = false;

    std::vector<double> q(numLinks,0),x,o;
    this->fwdKin(q,x,o);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::close() {
    delete _orient;
    CD_SUCCESS("Cleaned heap.\n");
    return true;
}

// -----------------------------------------------------------------------------


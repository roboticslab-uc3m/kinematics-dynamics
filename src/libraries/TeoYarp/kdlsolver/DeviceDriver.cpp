// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::KdlSolver::open(Searchable& config) {

    angleRepr = config.check("angleRepr",Value(DEFAULT_ANGLE_REPR),"axisAngle, eulerYZ, eulerZYZ or RPY").asString();

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

    yarp::sig::Matrix ymH0(4,4);
    ConstString ycsH0("H0");
    if( ! getMatrixFromProperties(config,ycsH0,ymH0) ){
        ymH0.eye();
        printf("KdlSolver using default H0: H0 = I\n");
    }
    else printf("KdlSolver using custom H0:\n%s\n",ymH0.toString().c_str());
    Vector kdlVec0(ymH0(0,3),ymH0(1,3),ymH0(2,3));
    Rotation kdlRot0( ymH0(0,0),ymH0(0,1),ymH0(0,2),ymH0(1,0),ymH0(1,1),ymH0(1,2),ymH0(2,0),ymH0(2,1),ymH0(2,2));
    theChain.addSegment(Segment(Joint(Joint::None), Frame(kdlRot0,kdlVec0)));

    for(int motor=0;motor<vectorOfCmcMotorIdxs.size();motor++) {

        std::string link("link_");
        std::ostringstream s;
        s << motor;
        link += s.str();
        Bottle &bLink = config.findGroup(link);
        if( ! bLink.isNull() ) {
            printf("KdlSolver using %s: ", link.c_str());
            double linkOffset = bLink.check("offset",Value(0.0)).asDouble();
            double linkD = bLink.check("D",Value(0.0)).asDouble();
            double linkA = bLink.check("A",Value(0.0)).asDouble();
            double linkAlpha = bLink.check("alpha",Value(0.0)).asDouble();
            theChain.addSegment(Segment(Joint(Joint::RotZ),Frame().DH(linkA,linkAlpha,linkD,linkOffset)));
            isPrismatic.push_back(0);
            printf(", offset: %f, D: %f, A: %f, alpha: %f.\n",linkOffset,linkD,linkA,linkAlpha);
            continue;
        }

        std::string xyzLink("xyzLink_");
        std::ostringstream xyzS;
        xyzS << motor;
        xyzLink += xyzS.str();
        CD_WARNING("Not found: \"%s\", looking for \"%s\" instead.\n", link.c_str(), xyzLink.c_str());
        Bottle &bXyzLink = config.findGroup(xyzLink);
        if( bXyzLink.isNull() ) {
            CD_ERROR("Not found: \"%s\" either.\n", xyzLink.c_str());
            return false;
        }
        printf("KdlSolver using %s: ", xyzLink.c_str());
        double linkX = bXyzLink.check("x",Value(0.0)).asDouble();
        double linkY = bXyzLink.check("y",Value(0.0)).asDouble();
        double linkZ = bXyzLink.check("z",Value(0.0)).asDouble();

        std::string linkType = bXyzLink.check("Type",Value("NULL")).asString();
        if(linkType == "RotX") {
            theChain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "RotY") {
            theChain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "RotZ") {
            theChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "InvRotX") {
            theChain.addSegment(Segment(Joint(Joint::RotX,-1.0),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "InvRotY") {
            theChain.addSegment(Segment(Joint(Joint::RotY,-1.0),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "InvRotZ") {
            theChain.addSegment(Segment(Joint(Joint::RotZ,-1.0),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(0);
        } else if(linkType == "TransX") {
            theChain.addSegment(Segment(Joint(Joint::TransX),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "TransY") {
            theChain.addSegment(Segment(Joint(Joint::TransY),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "TransZ") {
            theChain.addSegment(Segment(Joint(Joint::TransZ),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "InvTransX") {
            theChain.addSegment(Segment(Joint(Joint::TransX,-1.0),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "InvTransY") {
            theChain.addSegment(Segment(Joint(Joint::TransY,-1.0),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else if(linkType == "InvTransZ") {
            theChain.addSegment(Segment(Joint(Joint::TransZ,-1.0),Frame(Vector(linkX,linkY,linkZ))));
            isPrismatic.push_back(1);
        } else {
            CD_WARNING("Link joint type \"%s\" unrecognized!\n",linkType.c_str());
        }

        printf("%s, x: %f, y: %f, z: %f.\n",linkType.c_str(),linkX,linkY,linkZ);
    }

    yarp::sig::Matrix ymHN(4,4);
    ConstString ycsHN("HN");
    if( ! getMatrixFromProperties(config,ycsHN,ymHN) ){
        ymHN.eye();
        printf("KdlSolver using default HN: HN = I\n");
    }
    else printf("KdlSolver using custom HN:\n%s\n",ymHN.toString().c_str());
    Vector kdlVecN(ymHN(0,3),ymHN(1,3),ymHN(2,3));
    Rotation kdlRotN( ymHN(0,0),ymHN(0,1),ymHN(0,2),ymHN(1,0),ymHN(1,1),ymHN(1,2),ymHN(2,0),ymHN(2,1),ymHN(2,2));
    theChain.addSegment(Segment(Joint(Joint::None), Frame(kdlRotN,kdlVecN)));
    
    printf("KdlSolver chain number of segments including none-joint (H0 and HN): %d\n",theChain.getNrOfSegments());

    _orient = new RotationalInterpolation_SingleAxis();
    _eqradius = 1; //0.000001;
    _aggregate = false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::close() {
    delete _orient;
    CD_SUCCESS("Cleaned heap.\n");
    return true;
}

// -----------------------------------------------------------------------------


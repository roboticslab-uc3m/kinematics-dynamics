// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::KdlSolver::open(Searchable& config) {

    numLinks = config.check("numLinks",Value(DEFAULT_NUM_LINKS),"chain number of segments").asInt();
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

    for(int linkIndex=0;linkIndex<numLinks;linkIndex++) {

        std::string link("link_");
        std::ostringstream s;
        s << linkIndex;
        link += s.str();
        Bottle &bLink = config.findGroup(link);
        if( ! bLink.isNull() ) {
            double linkOffset = bLink.check("offset",Value(0.0)).asDouble();
            double linkD = bLink.check("D",Value(0.0)).asDouble();
            double linkA = bLink.check("A",Value(0.0)).asDouble();
            double linkAlpha = bLink.check("alpha",Value(0.0)).asDouble();
            theChain.addSegment(Segment(Joint(Joint::RotZ),Frame().DH(linkA,toRad(linkAlpha),linkD,toRad(linkOffset))));
            isPrismatic.push_back(0);
            CD_SUCCESS("Added: %s (offset %f) (D %f) (A %f) (alpha %f)\n",link.c_str(), linkOffset,linkD,linkA,linkAlpha);
            continue;
        }

        std::string xyzLink("xyzLink_");
        std::ostringstream xyzS;
        xyzS << linkIndex;
        xyzLink += xyzS.str();
        CD_WARNING("Not found: \"%s\", looking for \"%s\" instead.\n", link.c_str(), xyzLink.c_str());
        Bottle &bXyzLink = config.findGroup(xyzLink);
        if( bXyzLink.isNull() ) {
            CD_ERROR("Not found: \"%s\" either.\n", xyzLink.c_str());
            return false;
        }
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

        CD_SUCCESS("Added: %s (Type %s) (x %f) (y %f) (z %f)\n",xyzLink.c_str(),linkType.c_str(),linkX,linkY,linkZ);
    }

    printf("KdlSolver chain number of segments: %d\n",theChain.getNrOfSegments());

    _orient = new RotationalInterpolation_SingleAxis();
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


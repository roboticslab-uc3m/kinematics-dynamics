// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpTinyMath.hpp"

// ----------------------------------------------------------------------------

double teo::toDeg(const double inRad) {
    return (inRad * 180.0 / M_PI);  // return (inRad * 180.0 / 3.14159265);
}

// ----------------------------------------------------------------------------

double teo::toRad(const double inDeg) {
    return (inDeg * M_PI / 180.0);  // return (inDeg * 3.14159265 / 180.0);
}

// ----------------------------------------------------------------------------

void teo::xUpdateH(const yarp::sig::Vector &x, yarp::sig::Matrix &H) {
    H(0,3) = x(0);
    H(1,3) = x(1);
    H(2,3) = x(2);
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix teo::rotX(const double &inDeg) {
    yarp::sig::Matrix R = eye(3,3);
    double c=cos(toRad(inDeg));
    double s=sin(toRad(inDeg));
/*    R(1,1)=c;
    R(1,2)=s;
    R(2,1)=-s;
    R(2,2)=c;*/  // http://www.kwon3d.com/theory/transform/rot.html
    R(1,1)=c;
    R(1,2)=-s;
    R(2,1)=s;
    R(2,2)=c;
//printf("Rx:\n%s\n\n",R.toString().c_str());
    return R;  // yarp::sig::Matrix
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix teo::rotY(const double &inDeg) {
    yarp::sig::Matrix R = eye(3,3);
    double c=cos(toRad(inDeg));
    double s=sin(toRad(inDeg));
/*    R(0,0)=c;
    R(0,2)=-s;
    R(2,0)=s;
    R(2,2)=c;*/
    R(0,0)=c;
    R(0,2)=s;
    R(2,0)=-s;
    R(2,2)=c;
//printf("Ry:\n%s\n\n",R.toString().c_str());
    return R;  // yarp::sig::Matrix
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix teo::rotZ(const double &inDeg) {
    yarp::sig::Matrix R = eye(3,3);
    double c=cos(toRad(inDeg));
    double s=sin(toRad(inDeg));
    /*R(0,0)=c;
    R(0,1)=s;
    R(1,0)=-s;
    R(1,1)=c;*/
    R(0,0)=c;
    R(0,1)=-s;
    R(1,0)=s;
    R(1,1)=c;
//printf("Rz:\n%s\n\n",R.toString().c_str());
    return R;  // yarp::sig::Matrix
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix teo::eulerZYZtoH(const yarp::sig::Vector &x, const yarp::sig::Vector &o) {
    yarp::sig::Matrix result = rotZ(o[0]) * rotY(o[1]) * rotZ(o[2]);  // 3x3 
//printf("R:\n%s\n\n",result.toString().c_str());
    result.resize(4,4);
    result(3,3)=1;
    xUpdateH(x,result);
    return result;
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix teo::eulerYZtoH(const yarp::sig::Vector &x, const yarp::sig::Vector &o) {

    yarp::sig::Vector oZYZ(3);
    oZYZ[0] = toDeg(atan2(x[1],x[0]));
    oZYZ[1] = o(0);
    oZYZ[2] = o(1);

    return eulerZYZtoH(x,oZYZ);  // yarp::sig::Matrix
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix teo::axisAngleToH(const yarp::sig::Vector &x, const yarp::sig::Vector &o) {

    yarp::sig::Matrix H = eye(4,4);

    double theta = o[3];

    double c=cos(theta);
    double s=sin(theta);
    double C=1.0-c;

    double xs =o[0]*s;
    double ys =o[1]*s;
    double zs =o[2]*s;
    double xC =o[0]*C;
    double yC =o[1]*C;
    double zC =o[2]*C;
    double xyC=o[0]*yC;
    double yzC=o[1]*zC;
    double zxC=o[2]*xC;
    
    H(0,0)=o[0]*xC+c;
    H(0,1)=xyC-zs;
    H(0,2)=zxC+ys;
    H(1,0)=xyC+zs;
    H(1,1)=o[1]*yC+c;
    H(1,2)=yzC-xs;
    H(2,0)=zxC-ys;
    H(2,1)=yzC+xs;
    H(2,2)=o[2]*zC+c;
    
    xUpdateH(x,H);

    return H;
}

// ----------------------------------------------------------------------------

/* old impl
 * yarp::sig::Matrix asibot2h(const yarp::sig::Vector &x, const yarp::sig::Vector &o, unsigned int verbose) {

    // Form the axis-angle
    yarp::sig::Vector oAA;
    oAA.push_back(x[0]);
    oAA.push_back(x[1]);
    oAA.push_back( sqrt(x[0]*x[0]+x[1]*x[1]) * cos(toRad(o[0])) );
    oAA.push_back(toRad(o[1]));
    double length = sqrt( oAA[0]*oAA[0] + oAA[1]*oAA[1] + oAA[2]*oAA[2] );
    oAA[0] /= length; // normalize
    oAA[1] /= length; // normalize
    oAA[2] /= length; // normalize

    yarp::sig::Matrix H = axis2dcm(oAA);  // Returns a 3x3
    H.resize(4,4);
    H(3,0) = x(0);
    H(3,1) = x(1);
    H(3,2) = x(2);
    H(3,3) = 1;

    return H;
}*/

// ----------------------------------------------------------------------------


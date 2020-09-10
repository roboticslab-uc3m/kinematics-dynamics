// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpTinyMath.hpp"

#include <yarp/math/Math.h>  // provides: eye, operators

// ----------------------------------------------------------------------------

double roboticslab::toDeg(const double inRad)
{
    return (inRad * 180.0 / M_PI);  // return (inRad * 180.0 / 3.14159265);
}

// ----------------------------------------------------------------------------

double roboticslab::toRad(const double inDeg)
{
    return (inDeg * M_PI / 180.0);  // return (inDeg * 3.14159265 / 180.0);
}

// ----------------------------------------------------------------------------

void roboticslab::xUpdateH(const yarp::sig::Vector &x, yarp::sig::Matrix &H)
{
    H(0, 3) = x(0);
    H(1, 3) = x(1);
    H(2, 3) = x(2);
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix roboticslab::rotX(const double &inDeg)
{
    yarp::sig::Matrix R = yarp::math::eye(3, 3);
    double c = std::cos(toRad(inDeg));
    double s = std::sin(toRad(inDeg));
    // http://www.kwon3d.com/theory/transform/rot.html
    R(1, 1) = c;
    R(1, 2) = -s;
    R(2, 1) = s;
    R(2, 2) = c;
    return R;
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix roboticslab::rotY(const double &inDeg)
{
    yarp::sig::Matrix R = yarp::math::eye(3, 3);
    double c = std::cos(toRad(inDeg));
    double s = std::sin(toRad(inDeg));
    // http://www.kwon3d.com/theory/transform/rot.html
    R(0, 0) = c;
    R(0, 2) = s;
    R(2, 0) = -s;
    R(2, 2) = c;
    return R;
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix roboticslab::rotZ(const double &inDeg)
{
    yarp::sig::Matrix R = yarp::math::eye(3, 3);
    double c = std::cos(toRad(inDeg));
    double s = std::sin(toRad(inDeg));
    // http://www.kwon3d.com/theory/transform/rot.html
    R(0, 0) = c;
    R(0, 1) = -s;
    R(1, 0) = s;
    R(1, 1) = c;
    return R;
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix roboticslab::eulerZYZtoH(const yarp::sig::Vector &x, const yarp::sig::Vector &o)
{
    yarp::sig::Matrix result = rotZ(o[0]) * rotY(o[1]) * rotZ(o[2]);  // 3x3 
    result.resize(4, 4);
    result(3, 3) = 1;
    xUpdateH(x, result);
    return result;
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix roboticslab::eulerYZtoH(const yarp::sig::Vector &x, const yarp::sig::Vector &o)
{
    yarp::sig::Vector oZYZ(3);
    oZYZ[0] = toDeg(std::atan2(x[1], x[0]));
    oZYZ[1] = o(0);
    oZYZ[2] = o(1);
    return eulerZYZtoH(x, oZYZ);
}

// ----------------------------------------------------------------------------

yarp::sig::Matrix roboticslab::axisAngleToH(const yarp::sig::Vector &x, const yarp::sig::Vector &o)
{
    yarp::sig::Matrix H = yarp::math::eye(4, 4);

    double theta = o[3];

    double c = std::cos(theta);
    double s = std::sin(theta);
    double C = 1.0 - c;

    double xs = o[0] * s;
    double ys = o[1] * s;
    double zs = o[2] * s;
    double xC = o[0] * C;
    double yC = o[1] * C;
    double zC = o[2] * C;
    double xyC = o[0] * yC;
    double yzC = o[1] * zC;
    double zxC = o[2] * xC;
    
    H(0, 0) = o[0] * xC + c;
    H(0, 1) = xyC - zs;
    H(0, 2) = zxC + ys;
    H(1, 0) = xyC + zs;
    H(1, 1) = o[1] * yC + c;
    H(1, 2) = yzC - xs;
    H(2, 0) = zxC - ys;
    H(2, 1) = yzC + xs;
    H(2, 2) = o[2] *zC + c;
    
    xUpdateH(x, H);

    return H;
}

// ----------------------------------------------------------------------------

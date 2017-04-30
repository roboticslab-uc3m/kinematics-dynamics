// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_TINY_MATH_HPP__
#define __YARP_TINY_MATH_HPP__

#define _USE_MATH_DEFINES // see <math.h> on Windows
#include <math.h>  // provides: M_PI

#include <stdio.h>  // provides: fprintf, stderr

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>  // provides: eye, operators

namespace teo
{

using namespace yarp::math;  // else matrix operators wreak havoc

/**
 * @ingroup kinematics-dynamics-libraries
 * @defgroup TinyMath
 * @brief A small mathematical library, mostly for performing transformations between angle representations.
 */

/**
 * Simple function to pass from radians to degrees.
 * @param inRad angle value in radians.
 * @return angle value in degrees.
 */
double toDeg(const double inRad);

/**
 * Simple function to pass from degrees to radians.
 * @param inDeg angle value in degrees.
 * @return angle value in radians.
 */
double toRad(const double inDeg);

/**
 * Update x values of an homogeneous matrix.
 * @param x 3-vector input.
 * @param H homogeneous matrix (4x4) that will be modified.
 */
void xUpdateH(const yarp::sig::Vector &x, yarp::sig::Matrix &H);

yarp::sig::Matrix rotX(const double &inDeg);
yarp::sig::Matrix rotY(const double &inDeg);
yarp::sig::Matrix rotZ(const double &inDeg);

/**
 * @param x 3-vector in meters.
 * @param o 3-vector in degrees.
 * @return Homogeneous matrix (4x4).
 */
yarp::sig::Matrix eulerZYZtoH(const yarp::sig::Vector &x, const yarp::sig::Vector &o);

/**
 * Uses x to compute rot(Z).
 * @param x 3-vector in meters.
 * @param o 2-vector in degrees.
 * @return Homogeneous matrix (4x4).
 */
yarp::sig::Matrix eulerYZtoH(const yarp::sig::Vector &x, const yarp::sig::Vector &o);

/**
 * Thanks [Ugo Pattacini, Serena Ivaldi, Francesco Nori ((iCub ctrllib/math.h))] for axis2dcm().
 * @param x 3-vector in meters.
 * @param o 4-vector in degrees.
 * @return Homogeneous matrix (4x4).
 */
yarp::sig::Matrix axisAngleToH(const yarp::sig::Vector &x, const yarp::sig::Vector &o);

}  // namespace teo

#endif  // __YARP_TINY_MATH_HPP__


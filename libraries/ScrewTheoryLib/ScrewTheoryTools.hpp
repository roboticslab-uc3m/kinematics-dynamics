// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCREW_THEORY_TOOLS_HPP__
#define __SCREW_THEORY_TOOLS_HPP__

#include <cmath>

#include <kdl/frames.hpp>
#include <kdl/utilities/utility.h>

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-libraries
 * \defgroup ScrewTheoryLib
 *
 * @brief Contains classes related to Screw Theory solvers and tools.
 *
 * Proof of concept for a kinematics library based on
 * <a href="https://en.wikipedia.org/wiki/Screw_theory">screw theory</a>
 * concepts applied to robotics. This implementation is mainly focused
 * on solving inverse kinematics problems in an efficient and effective
 * manner via closed-form geometric solutions. Comparing this approach
 * with a tradicional numeric-based approach yields
 * (@cite pardosgotor2018str @cite pardosgotor2022str):
 *   - faster solutions, since there are no iterations involved;
 *   - exact solutions, since the direct formulation guarantees convergence;
 *   - multiple solutions and the possibility to choose the better ones.
 *
 * @see @cite lukawski2022jjaa
 */

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Multiply a vector by itself to obtain a square matrix
 *
 * Given a column-vector v, computes v*v'.
 *
 * @param v Input vector.
 *
 * @return Resulting square matrix.
 */
KDL::Rotation vectorPow2(const KDL::Vector & v);

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Clip an angle value between certain bounds.
 *
 * Values that exceed [-pi pi] are normalized to fit this range. Values
 * close to -pi are approximated as pi.
 *
 * @param angle Magnitude of the requested angle.
 *
 * @return Normalized angle.
 */
double normalizeAngle(double angle);

} // namespace roboticslab

#endif // __SCREW_THEORY_TOOLS_HPP__

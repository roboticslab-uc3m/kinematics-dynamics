// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCREW_THEORY_TOOLS_HPP__
#define __SCREW_THEORY_TOOLS_HPP__

#include <kdl/frames.hpp>
#include <kdl/utilities/utility.h>

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-libraries
 * \defgroup ScrewTheoryLib
 *
 * @brief Contains classes related to Screw Theory solvers and tools.
 */

inline KDL::Rotation vectorPow2(const KDL::Vector & v)
{
    return KDL::Rotation(v.x() * v.x(), v.x() * v.y(), v.x() * v.z(),
                         v.x() * v.y(), v.y() * v.y(), v.y() * v.z(),
                         v.x() * v.z(), v.y() * v.z(), v.z() * v.z());
}

inline double normalizeAngle(double angle)
{
    if (angle > KDL::PI)
    {
        return angle - 2 * KDL::PI;
    }
    else if (angle <= -KDL::PI)
    {
        return angle + 2 * KDL::PI;
    }
    else
    {
        return angle;
    }
}

}  // namespace roboticslab

#endif  // __SCREW_THEORY_TOOLS_HPP__

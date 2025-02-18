// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryTools.hpp"

#include <cmath>

#include <kdl/utilities/utility.h>

// -----------------------------------------------------------------------------

KDL::Rotation roboticslab::vectorPow2(const KDL::Vector & v)
{
    return KDL::Rotation(v.x() * v.x(), v.x() * v.y(), v.x() * v.z(),
                         v.x() * v.y(), v.y() * v.y(), v.y() * v.z(),
                         v.x() * v.z(), v.y() * v.z(), v.z() * v.z());
}

// -----------------------------------------------------------------------------

double roboticslab::normalizeAngle(double angle)
{
    if (KDL::Equal(std::abs(angle), KDL::PI))
    {
        return KDL::PI;
    }
    else if (angle > KDL::PI)
    {
        return angle - 2 * KDL::PI;
    }
    else if (angle < -KDL::PI)
    {
        return angle + 2 * KDL::PI;
    }
    else
    {
        return angle;
    }
}

// -----------------------------------------------------------------------------

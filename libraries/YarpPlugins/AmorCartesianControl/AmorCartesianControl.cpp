// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <cmath>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::checkJointVelocities(const std::vector<double> &qdot)
{
    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if (std::abs(qdot[i]) > maxJointVelocity)
        {
            CD_ERROR("Maximum angular velocity hit: qdot[%d] = %f > %f [deg/s].\n", i, qdot[i], maxJointVelocity);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

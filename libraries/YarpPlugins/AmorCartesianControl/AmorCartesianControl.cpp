// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <cmath>

#include <yarp/os/Log.h>

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::checkJointVelocities(const std::vector<double> &qdot)
{
    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if (std::abs(qdot[i]) > qdotMax[i])
        {
            yError("Maximum angular velocity hit: qdot[%d] = %f > %f [deg/s]", i, qdot[i], qdotMax[i]);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

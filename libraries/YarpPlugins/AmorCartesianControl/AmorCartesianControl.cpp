// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <cmath>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool AmorCartesianControl::checkJointVelocities(const std::vector<double> & qdot)
{
    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if (std::abs(qdot[i]) > qdotMax[i])
        {
            yCError(AMOR, "Maximum angular velocity hit: qdot[%d] = %f > %f [deg/s]", i, qdot[i], qdotMax[i]);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

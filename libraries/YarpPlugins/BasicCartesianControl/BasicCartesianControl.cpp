// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

namespace
{
    double epsilon = 1e-5;
}

// -----------------------------------------------------------------------------

int roboticslab::BasicCartesianControl::getCurrentState() const
{
    int tmp;
    currentStateReady.wait();
    tmp = currentState;
    currentStateReady.post();

    return tmp;
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::setCurrentState(int value)
{
    currentStateReady.wait();
    currentState = value;
    currentStateReady.post();
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::checkJointLimits(const std::vector<double> &q)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        double value = q[joint];

        // Report limit before reaching the actual value.
        // https://github.com/roboticslab-uc3m/kinematics-dynamics/issues/161#issuecomment-428133287
        if (value < qMin[joint] + epsilon || value > qMax[joint] - epsilon)
        {
            CD_WARNING("Joint near or out of limits: q[%d] = %f not in [%f,%f].\n", joint, value, qMin[joint], qMax[joint]);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::checkJointVelocities(const std::vector<double> &qdot)
{
    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if (std::abs(qdot[i]) > maxJointVelocity)
        {
            CD_WARNING("Maximum angular velocity hit: qdot[%d] = %f > %f [deg/s].\n", i, qdot[i], maxJointVelocity);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::setControlModes(int mode)
{
    if (currentControlMode != mode)
    {
        std::vector<int> modes(numRobotJoints, mode);

        if (!iControlMode->setControlModes(modes.data()))
        {
            CD_WARNING("setControlModes failed.\n");
            return false;
        }

        currentControlMode = mode;
    }

    return true;
}

// -----------------------------------------------------------------------------

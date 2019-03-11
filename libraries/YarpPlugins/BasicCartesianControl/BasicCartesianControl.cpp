// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

namespace
{
    double epsilon = 1e-5;

    // return -1 for negative numbers, +1 for positive numbers, 0 for zero
    // https://stackoverflow.com/a/4609795
    template <typename T>
    inline int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
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
    streamingCommand = DEFAULT_STREAMING_PRESET;
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

bool roboticslab::BasicCartesianControl::checkJointLimits(const std::vector<double> &q, const std::vector<double> &qd)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        double value = q[joint];

        if (value < qMin[joint] || value > qMax[joint])
        {
            CD_WARNING("Joint near or out of limits: q[%d] = %f not in [%f,%f].\n", joint, value, qMin[joint], qMax[joint]);
            double midRange = (qMax[joint] + qMin[joint]) / 2;

            // Let the joint get away from its nearest limit.
            if (qd.empty() || sgn(value - midRange) == sgn(qd[joint]))
            {
                return false;
            }
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
            CD_WARNING("Maximum angular velocity hit: qdot[%d] = |%f| > %f [deg/s].\n", i, qdot[i], maxJointVelocity);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::setControlModes(int mode)
{
    std::vector<int> modes(numRobotJoints);

    if (!iControlMode->getControlModes(modes.data()))
    {
        CD_WARNING("getControlModes failed.\n");
        return false;
    }

    std::vector<int> jointIds;

    for (unsigned int i = 0; i < modes.size(); i++)
    {
        if (modes[i] != mode)
        {
            jointIds.push_back(i);
        }
    }

    if (!jointIds.empty())
    {
        modes.assign(jointIds.size(), mode);

        if (!iControlMode->setControlModes(jointIds.size(), jointIds.data(), modes.data()))
        {
            CD_WARNING("setControlModes failed.\n");
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::presetStreamingCommand(int command)
{
    setCurrentState(VOCAB_CC_NOT_CONTROLLING);

    bool ret;

    switch (command)
    {
    case VOCAB_CC_TWIST:
        ret = setControlModes(VOCAB_CM_VELOCITY);
        break;
    case VOCAB_CC_POSE:
        ret = setControlModes(VOCAB_CM_VELOCITY);
        break;
    case VOCAB_CC_MOVI:
        ret = setControlModes(VOCAB_CM_POSITION_DIRECT);
        break;
    default:
        CD_ERROR("Unrecognized or unsupported streaming command vocab.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

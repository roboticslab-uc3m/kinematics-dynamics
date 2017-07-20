// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotConfiguration.hpp"

#include <ColorDebug.hpp>

using namespace roboticslab;

namespace
{
    inline bool checkJointInLimit(double q, double qMin, double qMax)
    {
        return q >= qMin && q <= qMax;
    }
}

bool AsibotConfiguration::configure(double q1, double q2u, double q2d, double q3, double q4, double q5)
{
    forwardElbowUp.storeAngles(q1, q2u, q3, q4, q5);

    if (!forwardElbowUp.checkJointsInLimits(_qMin, _qMax))
    {
        forwardElbowUp.valid = false;
    }

    forwardElbowDown.storeAngles(q1, q2d, -q3, q4, q5);

    if (!forwardElbowDown.checkJointsInLimits(_qMin, _qMax))
    {
        forwardElbowDown.valid = false;
    }

    reversedElbowUp.storeAngles(-q1, -q2u, -q3, -q4, -q5);

    if (!reversedElbowUp.checkJointsInLimits(_qMin, _qMax))
    {
        reversedElbowUp.valid = false;
    }

    reversedElbowDown.storeAngles(-q1, -q2d, q3, -q4, -q5);

    if (!reversedElbowDown.checkJointsInLimits(_qMin, _qMax))
    {
        reversedElbowDown.valid = false;
    }

    return forwardElbowUp.valid || forwardElbowDown.valid || reversedElbowUp.valid || reversedElbowDown.valid;
}

void AsibotConfiguration::AsibotPose::storeAngles(double q1, double q2, double q3, double q4, double q5)
{
    _q1 = q1;
    _q2 = q2;
    _q3 = q3;
    _q4 = q4;
    _q5 = q5;
}

bool AsibotConfiguration::AsibotPose::checkJointsInLimits(JointsIn qMin, JointsIn qMax) const
{
    double joints[5] = {_q1, _q2, _q3, _q4, _q5};
    bool ok = true;

    for (int i = 0; i < 5; i++)
    {
        double joint = joints[i];

        if (!checkJointInLimit(joint, qMin[i], qMax[i]))
        {
            CD_WARNING("Joint %d out of limits: %f not in [%f, %f].\n", i + 1, joint, qMin[i], qMax[i]);
            ok = false;
        }
    }

    return ok;
}

void AsibotConfiguration::AsibotPose::retrieveAngles(JointsOut q) const
{
    q.resize(5);

    q[0] = _q1;
    q[1] = _q2;
    q[2] = _q3;
    q[3] = _q4;
    q[4] = _q5;
}

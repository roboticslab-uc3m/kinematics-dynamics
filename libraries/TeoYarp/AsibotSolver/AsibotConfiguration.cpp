// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotConfiguration.hpp"

#include <sstream>

#include <ColorDebug.hpp>

using namespace roboticslab;

namespace
{
    inline bool checkJointInLimit(double q, double qMin, double qMax)
    {
        return q >= qMin && q <= qMax;
    }

    double normalizeAngle(double q)
    {
        if (q > 180)
        {
            return q - 360;
        }
        else if (q <= -180)
        {
            return q + 360;
        }
        else
        {
            return q;
        }
    }
}

bool AsibotConfiguration::configure(double q1, double q2u, double q2d, double q3, double q4, double q5)
{
    forwardElbowUp.storeAngles(q1, q2u, q3, q4, q5, Pose::FORWARD, Pose::UP);

    if (!forwardElbowUp.checkJointsInLimits(_qMin, _qMax))
    {
        forwardElbowUp.valid = false;
    }

    CD_INFO("%s\n", forwardElbowUp.toString().c_str());

    forwardElbowDown.storeAngles(q1, q2d, -q3, -q4, q5, Pose::FORWARD, Pose::DOWN);

    if (!forwardElbowDown.checkJointsInLimits(_qMin, _qMax))
    {
        forwardElbowDown.valid = false;
    }

    CD_INFO("%s\n", forwardElbowDown.toString().c_str());

    reversedElbowUp.storeAngles(180 - q1, -q2u, -q3, -q4, 180 - q5, Pose::REVERSED, Pose::UP);

    if (!reversedElbowUp.checkJointsInLimits(_qMin, _qMax))
    {
        reversedElbowUp.valid = false;
    }

    CD_INFO("%s\n", reversedElbowUp.toString().c_str());

    reversedElbowDown.storeAngles(180 - q1, -q2d, q3, q4, 180 - q5, Pose::REVERSED, Pose::DOWN);

    if (!reversedElbowDown.checkJointsInLimits(_qMin, _qMax))
    {
        reversedElbowDown.valid = false;
    }

    CD_INFO("%s\n", reversedElbowDown.toString().c_str());

    return forwardElbowUp.valid || forwardElbowDown.valid || reversedElbowUp.valid || reversedElbowDown.valid;
}

void AsibotConfiguration::Pose::storeAngles(double q1, double q2, double q3, double q4, double q5, orientation orient, elbow elb)
{
    _q1 = normalizeAngle(q1);
    _q2 = normalizeAngle(q2);
    _q3 = normalizeAngle(q3);
    _q4 = normalizeAngle(q4);
    _q5 = normalizeAngle(q5);

    _orient = orient;
    _elb = elb;
}

bool AsibotConfiguration::Pose::checkJointsInLimits(JointsIn qMin, JointsIn qMax) const
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

void AsibotConfiguration::Pose::retrieveAngles(JointsOut q) const
{
    q.resize(5);

    q[0] = _q1;
    q[1] = _q2;
    q[2] = _q3;
    q[3] = _q4;
    q[4] = _q5;
}

std::string AsibotConfiguration::Pose::toString()
{
    std::stringstream ss;

    ss << (_orient == FORWARD ? "FORWARD" : "REVERSED");
    ss << " ";
    ss << (_elb == UP ? "UP" : "DOWN");
    ss << ": " << _q1 << " " << _q2 << " " << _q3 << " " << _q4 << " " << _q5;

    return ss.str();
}

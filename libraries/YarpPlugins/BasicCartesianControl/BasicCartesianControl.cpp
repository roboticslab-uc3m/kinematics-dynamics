// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath>

#include <algorithm>

#include <yarp/conf/version.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Vocab.h>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    // return -1 for negative numbers, +1 for positive numbers, 0 for zero
    // https://stackoverflow.com/a/4609795
    template <typename T>
    inline int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
}

constexpr double epsilon = 1e-5;

// -----------------------------------------------------------------------------

bool BasicCartesianControl::checkJointLimits(const std::vector<double> &q)
{
    for (unsigned int joint = 0; joint < numJoints; joint++)
    {
        double value = q[joint];

        // Report limit before reaching the actual value.
        // https://github.com/roboticslab-uc3m/kinematics-dynamics/issues/161#issuecomment-428133287
        if (value < qMin[joint] + epsilon || value > qMax[joint] - epsilon)
        {
            yCWarning(BCC, "Joint near or out of limits: q[%d] = %f not in [%f,%f] (deg)",
                      joint, value, qMin[joint], qMax[joint]);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::checkJointLimits(const std::vector<double> &q, const std::vector<double> &qdot)
{
    for (unsigned int joint = 0; joint < numJoints; joint++)
    {
        double value = q[joint];

        if (value < qMin[joint] + epsilon || value > qMax[joint] - epsilon)
        {
            yCWarning(BCC, "Joint near or out of limits: q[%d] = %f not in [%f,%f] (deg)",
                      joint, value, qMin[joint], qMax[joint]);
            double midRange = (qMax[joint] + qMin[joint]) / 2;

            // Let the joint get away from its nearest limit.
            if (sgn(value - midRange) == sgn(qdot[joint]))
            {
                return false;
            }
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::checkJointVelocities(const std::vector<double> &qdot)
{
    for (unsigned int joint = 0; joint < numJoints; joint++)
    {
        double value = qdot[joint];

        if (qdotMin[joint] == 0.0)
        {
            if (std::abs(value) > qdotMax[joint])
            {
                yCWarning(BCC, "Maximum angular velocity hit: |qdot[%d]| = %f exceeds %f (deg/s)",
                          joint, std::abs(value), qdotMax[joint]);
                return false;
            }
        }
        else if (value < qdotMin[joint] || value > qdotMax[joint])
        {
            yCWarning(BCC, "Maximum angular velocity hit: qdot[%d] = %f not in [%f,%f] (deg/s)",
                      joint, value, qdotMin[joint], qdotMax[joint]);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::doFailFastChecks(const std::vector<double> & initialQ)
{
    if (trajectories.empty())
    {
        yCWarning(BCC, "Fail-fast check requested, but no trajectories are stored");
        return false;
    }

    const double timestep = yarp::os::PeriodicThread::getPeriod() * 0.5;

    std::vector<double> x;
    std::size_t numTcps;

    iCartesianSolver->getNumTcps(numTcps);
    x.reserve(numTcps * 6);

    std::vector<double> oldQ = initialQ;
    std::vector<double> newQ;
    std::vector<double> qdot(numTcps * 6);

    double maxDuration = 0.0;

    for (const auto & trajectory : trajectories)
    {
        maxDuration = std::max(maxDuration, trajectory->Duration());
    }

    for (double interval = 0.0; interval <= maxDuration; interval += timestep)
    {
        x.clear(); // leaves the capacity of the vector unchanged

        for (const auto & trajectory : trajectories)
        {
            const KDL::Frame & H = trajectory->Pos(interval);
            const std::vector<double> & x_tcp = KdlVectorConverter::frameToVector(H);
            x.insert(x.end(), x_tcp.cbegin(), x_tcp.cend());
        }

        if (!iCartesianSolver->inverseKinematics(x, oldQ, newQ))
        {
            yCWarning(BCC) << "IK failed at interval" << interval << "out of" << maxDuration << "seconds";
            return false;
        }

        std::transform(newQ.cbegin(), newQ.cend(), oldQ.cbegin(), qdot.begin(), std::minus<double>());

        if (!checkJointLimits(newQ) || !checkJointVelocities(qdot))
        {
            yCWarning(BCC) << "Limits hit at interval" << interval << "out of" << maxDuration << "seconds";
            return false;
        }

        oldQ = newQ;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::checkControlModes(int mode)
{
#if YARP_VERSION_COMPARE(>=, 4,0,0)
    std::vector<yarp::dev::ControlModeEnum> modes(numJoints);
#else
    std::vector<int> modes(numJoints);
#endif

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    if (!iControlMode->getControlModes(modes))
#else
    if (!iControlMode->getControlModes(modes.data()))
#endif
    {
        yCWarning(BCC) << "getControlModes() failed";
        return false;
    }

    return std::all_of(modes.begin(), modes.end(), [mode](auto retrievedMode) { return static_cast<int>(retrievedMode) == mode; });
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::setControlModes(int mode)
{
#if YARP_VERSION_COMPARE(>=, 4,0,0)
    std::vector<yarp::dev::ControlModeEnum> modes(numJoints);
#else
    std::vector<int> modes(numJoints);
#endif

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    if (!iControlMode->getControlModes(modes))
#else
    if (!iControlMode->getControlModes(modes.data()))
#endif
    {
        yCWarning(BCC) << "getControlModes() failed";
        return false;
    }

    std::vector<int> jointIds;

    for (unsigned int i = 0; i < modes.size(); i++)
    {
        if (static_cast<int>(modes[i]) != mode)
        {
            jointIds.push_back(i);
        }
    }

    if (!jointIds.empty())
    {
#if YARP_VERSION_COMPARE(>=, 4,0,0)
        auto modeSet = static_cast<yarp::dev::SelectableControlModeEnum>(mode);
        std::vector modesSet(jointIds.size(), modeSet);
#else
        modes.assign(jointIds.size(), mode);
#endif

#if YARP_VERSION_COMPARE(>=, 4,0,0)
        if (!iControlMode->setControlModes(jointIds, modesSet))
#else
        if (!iControlMode->setControlModes(jointIds.size(), jointIds.data(), modes.data()))
#endif
        {
            yCWarning(BCC) << "setControlModes() failed for mode:" << yarp::os::Vocab32::decode(mode);
            return false;
        }
    }

    int retry = 0;

    do
    {
        yarp::os::SystemClock::delaySystem(0.1);

        if (checkControlModes(mode))
        {
            return true;
        }
    } while (retry++ < 10);

    yCWarning(BCC) << "Max retries exceeded for mode change:" << yarp::os::Vocab32::decode(mode);
    return false;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::presetStreamingCommand(Streaming command)
{
    currentMode = Mode::NONE;

    switch (command)
    {
    case Streaming::POSE:
        return setControlModes(VOCAB_CM_POSITION_DIRECT);
    case Streaming::TWIST:
        return setControlModes(VOCAB_CM_VELOCITY);
    case Streaming::WRENCH:
        return setControlModes(VOCAB_CM_TORQUE);
    default:
        yCError(BCC) << "Unrecognized or unsupported streaming command vocab:"
                     << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(command));
    }

    return false;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::computeIsocronousSpeeds(const std::vector<double> & q, const std::vector<double> & qd, std::vector<double> & qdot)
{
    std::vector<double> deltas(numJoints);
    double maxTime = 0.0;

    //-- Find out the maximum time to move

    for (int joint = 0; joint < numJoints; joint++)
    {
        if (qRefSpeeds[joint] <= 0.0)
        {
            yCWarning(BCC, "Zero or negative velocities sent at joint %d, not moving: %f", joint, qRefSpeeds[joint]);
            continue;
        }

        deltas[joint] = std::abs(qd[joint] - q[joint]);

        yCInfo(BCC, "Distance (joint %d): %f", joint, deltas[joint]);

        double targetTime = deltas[joint] / qRefSpeeds[joint];

        if (targetTime > maxTime)
        {
            maxTime = targetTime;
            yCInfo(BCC, "Candidate: %f", maxTime);
        }
    }

    //-- Compute, store old and set joint velocities given this time

    for (int joint = 0; joint < numJoints; joint++)
    {
        if (maxTime == 0.0)
        {
            qdot[joint] = 0.0;
            yCInfo(BCC, "qdot[%d] = 0.0 (forced)", joint);
        }
        else
        {
            qdot[joint] = deltas[joint] / maxTime;
            yCInfo(BCC, "qdot[%d] = %f", joint, qdot[joint]);
        }
    }

    maxTrajectoryDuration = maxTime;

    return maxTime != 0.0; // true: will move
}

// -----------------------------------------------------------------------------

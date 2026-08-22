// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

constexpr int MAX_ENCODER_ERRORS = 20;
constexpr double ENCODER_THROTTLE = 1.0; // [s]

// ------------------- PeriodicThread Related ------------------------------------

void BasicCartesianControl::run()
{
    const auto currentModeLocal = currentMode.load();

    if (currentModeLocal == Mode::NONE)
    {
        return;
    }

    StateWatcher watcher([this] { cmcSuccess = false; stopControl(); });

    std::vector<double> q(numJoints);
    std::vector<double> qdot(numJoints);
    std::vector<double> qdotdot(numJoints);

    if (!iEncoders->getEncoders(q.data()))
    {
        yCErrorThrottle(BCC, ENCODER_THROTTLE) << "Unable to query encoders";
        encoderErrors++;

        if (encoderErrors > MAX_ENCODER_ERRORS)
        {
            yCError(BCC) << "Exceeded maximum number of consecutive encoder read errors, aborting";
        }
        else
        {
            watcher.suppress();
        }

        return;
    }
    else if (!iEncoders->getEncoderSpeeds(qdot.data()) || !iEncoders->getEncoderAccelerations(qdotdot.data()))
    {
        yCWarningThrottle(BCC, ENCODER_THROTTLE) << "Unable to query encoder speeds or accelerations";
    }

    encoderErrors = 0; // reset error counter

    if (!checkJointLimits(q))
    {
        yCError(BCC) << "checkJointLimits() failed, stopping control";
        return;
    }

    switch (currentModeLocal)
    {
    case Mode::MOVEJ:
        handleMovj(q, watcher);
        break;
    case Mode::MOVEL:
        m_usePosdMovl ? handleMovlPosd(q, watcher) : handleMovlVel(q, watcher);
        break;
    case Mode::MOVEV:
        handleMovv(q, watcher);
        break;
    case Mode::GCMP:
        handleGcmp(q, watcher);
        break;
    case Mode::FORCE:
        handleForc(q, qdot, qdotdot, watcher);
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleMovj(const std::vector<double> &q, const StateWatcher & watcher)
{
    if (!checkControlModes(VOCAB_CM_POSITION))
    {
        yCError(BCC) << "Not in position control mode";
        return;
    }

    bool done;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iPositionControl->checkMotionDone(done))
#else
    if (!iPositionControl->checkMotionDone(&done))
#endif
    {
        yCError(BCC) << "Unable to query current robot state";
        return;
    }

    watcher.suppress();

    if (done)
    {
        currentMode = Mode::NONE;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        if (!iPositionControl->setTrajSpeeds(vmoStored.data()))
        {
             yCWarning(BCC) << "setTrajSpeeds() (to restore) failed";
        }
#else
        if (!iPositionControl->setRefSpeeds(vmoStored.data()))
        {
             yCWarning(BCC) << "setRefSpeeds() (to restore) failed";
        }
#endif
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleMovlVel(const std::vector<double> &q, const StateWatcher & watcher)
{
    if (!checkControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Not in velocity control mode";
        return;
    }

    double movementTime = yarp::os::Time::now() - movementStartTime;

    std::vector<double> desiredX, desiredXdot;

    for (const auto & trajectory : trajectories)
    {
        if (movementTime > trajectory->Duration())
        {
            watcher.suppress();
            stopControl();
            return;
        }

        //-- Obtain desired Cartesian position and velocity.
        KDL::Frame H = trajectory->Pos(movementTime);
        KDL::Twist tw = trajectory->Vel(movementTime);

        std::vector<double> desiredX_sub = KdlVectorConverter::frameToVector(H);
        std::vector<double> desiredXdot_sub = KdlVectorConverter::twistToVector(tw);

        desiredX.insert(desiredX.end(), desiredX_sub.cbegin(), desiredX_sub.cend());
        desiredXdot.insert(desiredXdot.end(), desiredXdot_sub.cbegin(), desiredXdot_sub.cend());
    }

    std::vector<double> currentX;

    if (!iCartesianSolver->forwardKinematics(q, currentX))
    {
        yCWarning(BCC) << "forwardKinematics() failed";
        return;
    }

    //-- Apply control law to compute robot Cartesian velocity commands.
    std::vector<double> commandXdot;
    iCartesianSolver->poseDiff(desiredX, currentX, commandXdot);

    for (unsigned int i = 0; i < commandXdot.size(); i++)
    {
        commandXdot[i] *= m_controllerGain * (1000.0 / m_cmcPeriodMs);
        commandXdot[i] += desiredXdot[i];
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;

    if (!iCartesianSolver->diffInverseKinematics(q, commandXdot, commandQdot))
    {
        yCWarning(BCC) << "diffInverseKinematics() failed";
        return;
    }

    yCDebug(BCC) << "[MOVEL]" << movementTime << "[s] ||" << commandXdot << "->" << commandQdot << "[deg/s]";

    if (!checkJointVelocities(commandQdot))
    {
        yCError(BCC) << "diffInverseKinematics() too dangerous";
        return;
    }

    watcher.suppress();

    if (!iVelocityControl->velocityMove(commandQdot.data()))
    {
        yCWarning(BCC) << "velocityMove() failed";
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleMovlPosd(const std::vector<double> &q, const StateWatcher & watcher)
{
    if (!checkControlModes(VOCAB_CM_POSITION_DIRECT))
    {
        yCError(BCC) << "Not in position direct control mode";
        return;
    }

    double movementTime = yarp::os::Time::now() - movementStartTime;

    std::vector<double> desiredX;

    for (const auto & trajectory : trajectories)
    {
        if (movementTime > trajectory->Duration())
        {
            watcher.suppress();
            stopControl();
            return;
        }

        //-- Obtain desired Cartesian position.
        KDL::Frame H = trajectory->Pos(movementTime);
        std::vector<double> desiredX_sub = KdlVectorConverter::frameToVector(H);
        desiredX.insert(desiredX.end(), desiredX_sub.cbegin(), desiredX_sub.cend());
    }

    //-- Compute joint position commands and send to robot.
    std::vector<double> commandQ;

    if (!iCartesianSolver->inverseKinematics(desiredX, q, commandQ))
    {
        yCWarning(BCC) << "inverseKinematics() failed";
        return;
    }

    yCDebug(BCC) << "[MOVEL]" << movementTime << "[s] ||" << desiredX << "->" << commandQ << "[deg]";

    watcher.suppress();

    if (!iPositionDirect->setPositions(commandQ.data()))
    {
        yCWarning(BCC) << "setPositions() failed";
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleMovv(const std::vector<double> &q, const StateWatcher & watcher)
{
    if (!checkControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Not in velocity control mode";
        return;
    }

    double movementTime = yarp::os::Time::now() - movementStartTime;

    std::vector<double> currentX;

    if (!iCartesianSolver->forwardKinematics(q, currentX))
    {
        yCWarning(BCC) << "forwardKinematics() failed";
        return;
    }

    //-- Obtain desired Cartesian position and velocity.
    std::vector<double> desiredX, desiredXdot;

    for (const auto & trajectory : trajectories)
    {
        //-- Obtain desired Cartesian position and velocity.
        KDL::Frame H = trajectory->Pos(movementTime);
        KDL::Twist tw = trajectory->Vel(movementTime);

        std::vector<double> desiredX_sub = KdlVectorConverter::frameToVector(H);
        std::vector<double> desiredXdot_sub = KdlVectorConverter::twistToVector(tw);

        desiredX.insert(desiredX.end(), desiredX_sub.cbegin(), desiredX_sub.cend());
        desiredXdot.insert(desiredXdot.end(), desiredXdot_sub.cbegin(), desiredXdot_sub.cend());
    }

    //-- Apply control law to compute robot Cartesian velocity commands.
    std::vector<double> commandXdot;
    iCartesianSolver->poseDiff(desiredX, currentX, commandXdot);

    for (unsigned int i = 0; i < commandXdot.size(); i++)
    {
        commandXdot[i] *= m_controllerGain * (1000.0 / m_cmcPeriodMs);
        commandXdot[i] += desiredXdot[i];
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;

    if (!iCartesianSolver->diffInverseKinematics(q, commandXdot, commandQdot, referenceFrame))
    {
        yCWarning(BCC) << "diffInverseKinematics() failed";
        return;
    }

    yCDebug(BCC) << "[MOVEV]" << movementTime << "[s] ||" << commandXdot << "->" << commandQdot << "[deg/s]";

    if (!checkJointVelocities(commandQdot))
    {
        yCError(BCC) << "diffInverseKinematics() too dangerous";
        return;
    }

    watcher.suppress();

    if (!iVelocityControl->velocityMove(commandQdot.data()))
    {
        yCWarning(BCC) << "velocityMove() failed";
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleGcmp(const std::vector<double> &q, const StateWatcher & watcher)
{
    if (!checkControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Not in torque control mode";
        return;
    }

    std::vector<double> t(numJoints);

    if (!iCartesianSolver->inverseDynamics(q, t))
    {
        yCWarning(BCC) << "inverseDynamics() failed";
        return;
    }

    watcher.suppress();

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        yCWarning(BCC) << "setRefTorques() failed";
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleForc(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot,
                                       const StateWatcher & watcher)
{
    if (!checkControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Not in torque control mode";
        return;
    }

    std::vector<double> t(numJoints);

    if (!iCartesianSolver->inverseDynamics(q, qdot, qdotdot, fd, t, referenceFrame))
    {
        yCWarning(BCC) << "inverseDynamics() failed";
        return;
    }

    watcher.suppress();

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        yCWarning(BCC) << "setRefTorques() failed";
    }
}

// -----------------------------------------------------------------------------

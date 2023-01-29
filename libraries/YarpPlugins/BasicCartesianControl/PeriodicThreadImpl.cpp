// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

namespace
{
    static constexpr int MAX_ENCODER_ERRORS = 20;
    static constexpr double ERROR_THROTTLE = 0.5; // [s]
}

// ------------------- PeriodicThread Related ------------------------------------

void BasicCartesianControl::run()
{
    const int currentState = getCurrentState();

    if (currentState == VOCAB_CC_NOT_CONTROLLING)
    {
        return;
    }

    StateWatcher watcher([this] { cmcSuccess = false; stopControl(); });

    std::vector<double> q(numJoints);
    std::vector<double> qdot(numJoints);
    std::vector<double> qdotdot(numJoints);

    if (!iEncoders->getEncoders(q.data()) || !iEncoders->getEncoderSpeeds(qdot.data()) || !iEncoders->getEncoderAccelerations(qdotdot.data()))
    {
        yCErrorThrottle(BCC, ERROR_THROTTLE) << "getEncoders() failed, unable to check joint limits";
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

    encoderErrors = 0; // reset error counter

    if (!checkJointLimits(q))
    {
        yCError(BCC) << "checkJointLimits() failed, stopping control";
        return;
    }

    switch (currentState)
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        handleMovj(q, watcher);
        break;
    case VOCAB_CC_MOVL_CONTROLLING:
        usePosdMovl ? handleMovlPosd(q, watcher) : handleMovlVel(q, watcher);
        break;
    case VOCAB_CC_MOVV_CONTROLLING:
        handleMovv(q, watcher);
        break;
    case VOCAB_CC_GCMP_CONTROLLING:
        handleGcmp(q, watcher);
        break;
    case VOCAB_CC_FORC_CONTROLLING:
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

    if (!iPositionControl->checkMotionDone(&done))
    {
        yCError(BCC) << "Unable to query current robot state";
        return;
    }

    watcher.suppress();

    if (done)
    {
        setCurrentState(VOCAB_CC_NOT_CONTROLLING);

        if (!iPositionControl->setRefSpeeds(vmoStored.data()))
        {
             yCWarning(BCC) << "setRefSpeeds() (to restore) failed";
        }
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

    if (!iCartesianSolver->fwdKin(q, currentX))
    {
        yCWarning(BCC) << "fwdKin() failed";
        return;
    }

    //-- Apply control law to compute robot Cartesian velocity commands.
    std::vector<double> commandXdot;
    iCartesianSolver->poseDiff(desiredX, currentX, commandXdot);

    for (unsigned int i = 0; i < commandXdot.size(); i++)
    {
        commandXdot[i] *= gain * (1000.0 / cmcPeriodMs);
        commandXdot[i] += desiredXdot[i];
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;

    if (!iCartesianSolver->diffInvKin(q, commandXdot, commandQdot))
    {
        yCWarning(BCC) << "diffInvKin() failed";
        return;
    }

    yCDebug(BCC) << "[MOVL]" << movementTime << "[s] ||" << commandXdot << "->" << commandQdot << "[deg/s]";

    if (!checkJointVelocities(commandQdot))
    {
        yCError(BCC) << "diffInvKin() too dangerous";
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

    if (!iCartesianSolver->invKin(desiredX, q, commandQ))
    {
        yCWarning(BCC) << "invKin() failed";
        return;
    }

    yCDebug(BCC) << "[MOVL]" << movementTime << "[s] ||" << desiredX << "->" << commandQ << "[deg]";

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

    if (!iCartesianSolver->fwdKin(q, currentX))
    {
        yCWarning(BCC) << "fwdKin() failed";
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
        commandXdot[i] *= gain * (1000.0 / cmcPeriodMs);
        commandXdot[i] += desiredXdot[i];
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;

    if (!iCartesianSolver->diffInvKin(q, commandXdot, commandQdot, referenceFrame))
    {
        yCWarning(BCC) << "diffInvKin() failed";
        return;
    }

    yCDebug(BCC) << "[MOVV]" << movementTime << "[s] ||" << commandXdot << "->" << commandQdot << "[deg/s]";

    if (!checkJointVelocities(commandQdot))
    {
        yCError(BCC) << "diffInvKin() too dangerous";
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

    if (!iCartesianSolver->invDyn(q, t))
    {
        yCWarning(BCC) << "invDyn() failed";
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

    if (!iCartesianSolver->invDyn(q, qdot, qdotdot, td, t, referenceFrame))
    {
        yCWarning(BCC) << "invDyn() failed";
        return;
    }

    watcher.suppress();

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        yCWarning(BCC) << "setRefTorques() failed";
    }
}

// -----------------------------------------------------------------------------

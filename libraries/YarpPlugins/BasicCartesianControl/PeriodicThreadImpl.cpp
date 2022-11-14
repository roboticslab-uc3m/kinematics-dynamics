// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- PeriodicThread Related ------------------------------------

void BasicCartesianControl::run()
{
    static constexpr int MAX_ENCODER_ERRORS = 20;
    static constexpr double ERROR_THROTTLE = 0.5; // [s]

    const int currentState = getCurrentState();

    if (currentState == VOCAB_CC_NOT_CONTROLLING)
    {
        return;
    }

    StateWatcher watcher([this] { cmcSuccess = false; stopControl(); });
    std::vector<double> q(numJoints);

    if (!iEncoders->getEncoders(q.data()))
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
        handleMovl(q, watcher);
        break;
    case VOCAB_CC_MOVV_CONTROLLING:
        handleMovv(q, watcher);
        break;
    case VOCAB_CC_GCMP_CONTROLLING:
        handleGcmp(q, watcher);
        break;
    case VOCAB_CC_FORC_CONTROLLING:
        handleForc(q, watcher);
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

void BasicCartesianControl::handleMovl(const std::vector<double> &q, const StateWatcher & watcher)
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

void BasicCartesianControl::handleForc(const std::vector<double> &q, const StateWatcher & watcher)
{
    if (!checkControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Not in torque control mode";
        return;
    }

    std::vector<double> qdot(numJoints), qdotdot(numJoints);
    std::vector< std::vector<double> > fexts;

    for (int i = 0; i < numJoints - 1; i++) //-- "numJoints - 1" is important
    {
        fexts.emplace_back(6, 0);
    }

    fexts.push_back(td);

    std::vector<double> t(numJoints);

    if (!iCartesianSolver->invDyn(q, qdot, qdotdot, fexts, t))
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
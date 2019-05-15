// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <yarp/os/Time.h>

#include <ColorDebug.h>

// ------------------- PeriodicThread Related ------------------------------------

void roboticslab::BasicCartesianControl::run()
{
    const int currentState = getCurrentState();

    if (currentState == VOCAB_CC_NOT_CONTROLLING)
    {
        return;
    }

    std::vector<double> q(numRobotJoints);

    if (!iEncoders->getEncoders(q.data()))
    {
        CD_ERROR("getEncoders failed, unable to check joint limits.\n");
        return;
    }

    if (!checkJointLimits(q))
    {
        CD_ERROR("checkJointLimits failed, stopping control.\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    switch (currentState)
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        handleMovj(q);
        break;
    case VOCAB_CC_MOVL_CONTROLLING:
        handleMovl(q);
        break;
    case VOCAB_CC_MOVV_CONTROLLING:
        handleMovv(q);
        break;
    case VOCAB_CC_GCMP_CONTROLLING:
        handleGcmp(q);
        break;
    case VOCAB_CC_FORC_CONTROLLING:
        handleForc(q);
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleMovj(const std::vector<double> &q)
{
    bool done;

    if (!iPositionControl->checkMotionDone(&done))
    {
        CD_ERROR("Unable to query current robot state.\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (done)
    {
        setCurrentState(VOCAB_CC_NOT_CONTROLLING);

        if (!iPositionControl->setRefSpeeds(vmoStored.data()))
        {
             CD_WARNING("setRefSpeeds (to restore) failed.\n");
        }
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleMovl(const std::vector<double> &q)
{
    double currentTrajectoryDuration;
    iCartesianTrajectory->getDuration(&currentTrajectoryDuration);

    double movementTime = yarp::os::Time::now() - movementStartTime;

    if (movementTime > currentTrajectoryDuration)
    {
        stopControl();
        return;
    }

    std::vector<double> currentX;

    if (!iCartesianSolver->fwdKin(q, currentX))
    {
        CD_WARNING("fwdKin failed, not updating control this iteration.\n");
        return;
    }

    //-- Obtain desired Cartesian position and velocity.
    std::vector<double> desiredX, desiredXdot;
    iCartesianTrajectory->getPosition(movementTime, desiredX);
    iCartesianTrajectory->getVelocity(movementTime, desiredXdot);

    //-- Apply control law to compute robot Cartesian velocity commands.
    std::vector<double> commandXdot;
    iCartesianSolver->poseDiff(desiredX, currentX, commandXdot);

    for (int i = 0; i < 6; i++)
    {
        commandXdot[i] *= gain * (1000.0 / cmcPeriodMs);
        commandXdot[i] += desiredXdot[i];
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;

    if (!iCartesianSolver->diffInvKin(q, commandXdot, commandQdot))
    {
        CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        return;
    }

    CD_DEBUG_NO_HEADER("[MOVL] [%f] ", movementTime);

    for (int i = 0; i < 6; i++)
    {
        CD_DEBUG_NO_HEADER("%f ", commandXdot[i]);
    }

    CD_DEBUG_NO_HEADER("-> ");

    for (int i = 0; i < numRobotJoints; i++)
    {
        CD_DEBUG_NO_HEADER("%f ", commandQdot[i]);
    }

    CD_DEBUG_NO_HEADER("[deg/s]\n");

    if (!checkJointVelocities(commandQdot))
    {
        CD_ERROR("diffInvKin too dangerous, STOP!!!\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (!iVelocityControl->velocityMove(commandQdot.data()))
    {
        CD_WARNING("velocityMove failed, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleMovv(const std::vector<double> &q)
{
    double movementTime = yarp::os::Time::now() - movementStartTime;

    std::vector<double> currentX;

    if (!iCartesianSolver->fwdKin(q, currentX))
    {
        CD_WARNING("fwdKin failed, not updating control this iteration.\n");
        return;
    }

    //-- Obtain desired Cartesian position and velocity.
    std::vector<double> desiredX, desiredXdot;

    trajectoryMutex.wait();
    iCartesianTrajectory->getPosition(movementTime, desiredX);
    iCartesianTrajectory->getVelocity(movementTime, desiredXdot);
    trajectoryMutex.post();

    //-- Apply control law to compute robot Cartesian velocity commands.
    std::vector<double> commandXdot;
    iCartesianSolver->poseDiff(desiredX, currentX, commandXdot);

    for (int i = 0; i < 6; i++)
    {
        commandXdot[i] *= gain * (1000.0 / cmcPeriodMs);
        commandXdot[i] += desiredXdot[i];
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;

    if (!iCartesianSolver->diffInvKin(q, commandXdot, commandQdot, referenceFrame))
    {
        CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        return;
    }

    CD_DEBUG_NO_HEADER("[MOVV] [%f] ", movementTime);

    for (int i = 0; i < 6; i++)
    {
        CD_DEBUG_NO_HEADER("%f ", commandXdot[i]);
    }

    CD_DEBUG_NO_HEADER("-> ");

    for (int i = 0; i < numRobotJoints; i++)
    {
        CD_DEBUG_NO_HEADER("%f ", commandQdot[i]);
    }

    CD_DEBUG_NO_HEADER("[deg/s]\n");

    if (!checkJointVelocities(commandQdot))
    {
        CD_ERROR("diffInvKin too dangerous, STOP!!!\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (!iVelocityControl->velocityMove(commandQdot.data()))
    {
        CD_WARNING("velocityMove failed, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleGcmp(const std::vector<double> &q)
{
    std::vector<double> t(numRobotJoints);

    if (!iCartesianSolver->invDyn(q, t))
    {
        CD_WARNING("invDyn failed, not updating control this iteration.\n");
        return;
    }

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        CD_WARNING("setRefTorques failed, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleForc(const std::vector<double> &q)
{
    std::vector<double> qdot(numRobotJoints, 0), qdotdot(numRobotJoints, 0);
    std::vector< std::vector<double> > fexts;

    for (int i = 0; i < numRobotJoints - 1; i++)  //-- "numRobotJoints-1" is important
    {
        std::vector<double> fext(6, 0);
        fexts.push_back(fext);
    }

    fexts.push_back(td);

    std::vector<double> t(numRobotJoints);

    if (!iCartesianSolver->invDyn(q, qdot, qdotdot, fexts, t))
    {
        CD_WARNING("invDyn failed, not updating control this iteration.\n");
        return;
    }

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        CD_WARNING("setRefTorques failed, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------

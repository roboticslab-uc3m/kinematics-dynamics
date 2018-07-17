// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath>  //-- abs
#include <yarp/os/Time.h>
#include <ColorDebug.h>

// ------------------- RateThread Related ------------------------------------

namespace
{
    std::vector<double> xRef(3, 0.0);
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::run()
{
    switch (getCurrentState())
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        handleMovj();
        break;
    case VOCAB_CC_MOVL_CONTROLLING:
        handleMovl();
        break;
    case VOCAB_CC_MOVV_CONTROLLING:
        handleMovv();
        break;
    case VOCAB_CC_GCMP_CONTROLLING:
        handleGcmp();
        break;
    case VOCAB_CC_FORC_CONTROLLING:
        handleForc();
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleMovj()
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

void roboticslab::BasicCartesianControl::handleMovl()
{
    double currentTrajectoryDuration;
    iCartesianTrajectory->getDuration(&currentTrajectoryDuration);

    double movementTime = yarp::os::Time::now() - movementStartTime;

    if (movementTime > currentTrajectoryDuration)
    {
        stopControl();
        return;
    }

    //-- Obtain current joint position
    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }

    std::vector<double> currentX;

    if (!iCartesianSolver->fwdKin(currentQ, currentX))
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
        commandXdot[i] *= gain * (1000.0 / cmcRateMs);
        commandXdot[i] += desiredXdot[i];
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;

    if (!iCartesianSolver->diffInvKin(currentQ, commandXdot, commandQdot))
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

    for (int i = 0; i < commandQdot.size(); i++)
    {
        if (std::abs(commandQdot[i]) > maxJointVelocity)
        {
            CD_ERROR("diffInvKin too dangerous, STOP!!!\n");
            cmcSuccess = false;
            stopControl();
            return;
        }
    }

    if (!iVelocityControl->velocityMove(commandQdot.data()))
    {
        CD_WARNING("velocityMove failed, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleMovv()
{
    //-- Obtain current joint position
    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;

    if (!iCartesianSolver->diffInvKin(currentQ, xdotd, commandQdot, referenceFrame))
    {
        CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        return;
    }

    CD_DEBUG_NO_HEADER("[MOVV] ");

    for (int i = 0; i < 6; i++)
    {
        CD_DEBUG_NO_HEADER("%f ", xdotd[i]);
    }

    CD_DEBUG_NO_HEADER("-> ");

    for (int i = 0; i < numRobotJoints; i++)
    {
        CD_DEBUG_NO_HEADER("%f ", commandQdot[i]);
    }

    CD_DEBUG_NO_HEADER("[deg/s]\n");

    for (int i = 0; i < commandQdot.size(); i++)
    {
        if (std::abs(commandQdot[i]) > maxJointVelocity)
        {
            CD_ERROR("diffInvKin too dangerous, STOP!!!\n");
            stopControl();
            return;
        }
    }

    if (!iVelocityControl->velocityMove(commandQdot.data()))
    {
        CD_WARNING("velocityMove failed, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleGcmp()
{
    //-- Obtain current joint position
    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }

    std::vector<double> t(numRobotJoints);

    if (stiffness == 0.0)
    {
        if (!iCartesianSolver->invDyn(currentQ, t))
        {
            CD_WARNING("invDyn failed, not updating control this iteration.\n");
            return;
        }
    }
    else
    {
        std::vector<double> currentX;

        if (!iCartesianSolver->fwdKin(currentQ, currentX))
        {
            CD_ERROR("fwdKin failed.\n");
            return;
        }

        if (!preservePose)
        {
            for (unsigned int i = 0; i < xRef.size(); i++)
            {
                xRef[i] = currentX[i];
            }

            preservePose = true;
        }

        std::vector<double> ftcp(6, 0.0);

        for (unsigned int i = 0; i < xRef.size(); i++)
        {
            ftcp[i] = (xRef[i] - currentX[i]) * stiffness;
        }

        std::vector< std::vector<double> > fexts;

        for (int i = 0; i < numRobotJoints - 1; i++)  //-- "numRobotJoints-1" is important
        {
            std::vector<double> fext(6, 0.0);
            fexts.push_back(fext);
        }

        fexts.push_back(ftcp);

        std::vector<double> qdot(numRobotJoints, 0.0);
        std::vector<double> qdotdot(numRobotJoints, 0.0);

        if (!iCartesianSolver->invDyn(currentQ, qdot, qdotdot, fexts, t))
        {
            CD_WARNING("invDyn failed, not updating control this iteration.\n");
            return;
        }
    }

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        CD_WARNING("setRefTorques failed, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::handleForc()
{
    //-- Obtain current joint position
    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }

    std::vector<double> qdot(numRobotJoints, 0), qdotdot(numRobotJoints, 0);
    std::vector< std::vector<double> > fexts;

    for (int i = 0; i < numRobotJoints - 1; i++)  //-- "numRobotJoints-1" is important
    {
        std::vector<double> fext(6, 0);
        fexts.push_back(fext);
    }

    fexts.push_back(td);

    std::vector<double> t(numRobotJoints);

    if (!iCartesianSolver->invDyn(currentQ, qdot, qdotdot, fexts, t))
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

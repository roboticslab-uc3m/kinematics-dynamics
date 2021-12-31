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
    const int currentState = getCurrentState();

    if (currentState == VOCAB_CC_NOT_CONTROLLING)
    {
        return;
    }

    std::vector<double> q(numRobotJoints);

    if (!iEncoders->getEncoders(q.data()))
    {
        yCError(BCC) << "getEncoders() failed, unable to check joint limits";
        return;
    }

    if (!checkJointLimits(q))
    {
        yCError(BCC) << "checkJointLimits() failed, stopping control";
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

void BasicCartesianControl::handleMovj(const std::vector<double> &q)
{
    if (!checkControlModes(VOCAB_CM_POSITION))
    {
        yCError(BCC) << "Not in position control mode";
        cmcSuccess = false;
        stopControl();
        return;
    }

    bool done;

    if (!iPositionControl->checkMotionDone(&done))
    {
        yCError(BCC) << "Unable to query current robot state";
        cmcSuccess = false;
        stopControl();
        return;
    }

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

void BasicCartesianControl::handleMovl(const std::vector<double> &q)
{
    if (!checkControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Not in velocity control mode";
        cmcSuccess = false;
        stopControl();
        return;
    }

    double movementTime = yarp::os::Time::now() - movementStartTime;

    std::vector<double> desiredX, desiredXdot;

    for (const auto & trajectory : trajectories)
    {
        if (movementTime > trajectory->Duration())
        {
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
        yCWarning(BCC) << "fwdKin() failed, not updating control this iteration";
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
        yCWarning(BCC) << "diffInvKin() failed, not updating control this iteration";
        return;
    }

    yCDebug(BCC) << "[MOVL]" << movementTime << "||" << commandXdot << "->" << commandQdot << "[deg/s]";

    if (!checkJointVelocities(commandQdot))
    {
        yCError(BCC) << "diffInvKin() too dangerous, stopping";
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (!iVelocityControl->velocityMove(commandQdot.data()))
    {
        yCWarning(BCC) << "velocityMove() failed, not updating control this iteration";
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleMovv(const std::vector<double> &q)
{
    if (!checkControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Not in velocity control mode";
        cmcSuccess = false;
        stopControl();
        return;
    }

    double movementTime = yarp::os::Time::now() - movementStartTime;

    std::vector<double> currentX;

    if (!iCartesianSolver->fwdKin(q, currentX))
    {
        yCWarning(BCC) << "fwdKin() failed, not updating control this iteration";
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
        yCWarning(BCC) << "diffInvKin() failed, not updating control this iteration";
        return;
    }

    yCDebug(BCC) << "[MOVV]" << movementTime << "||" << commandXdot << "->" << commandQdot << "[deg/s]";

    if (!checkJointVelocities(commandQdot))
    {
        yCError(BCC) << "diffInvKin() too dangerous, stopping";
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (!iVelocityControl->velocityMove(commandQdot.data()))
    {
        yCWarning(BCC) << "velocityMove() failed, not updating control this iteration";
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleGcmp(const std::vector<double> &q)
{
    if (!checkControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Not in torque control mode";
        stopControl();
        return;
    }

    std::vector<double> t(numRobotJoints);

    if (!iCartesianSolver->invDyn(q, t))
    {
        yCWarning(BCC) << "invDyn() failed, not updating control this iteration";
        return;
    }

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        yCWarning(BCC) << "setRefTorques() failed, not updating control this iteration";
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::handleForc(const std::vector<double> &q)
{
    if (!checkControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Not in torque control mode";
        stopControl();
        return;
    }

    std::vector<double> qdot(numRobotJoints, 0), qdotdot(numRobotJoints, 0);
    std::vector< std::vector<double> > fexts;

    for (int i = 0; i < numRobotJoints - 1; i++)  //-- "numRobotJoints-1" is important
    {
        fexts.emplace_back(6, 0);
    }

    fexts.push_back(td);

    std::vector<double> t(numRobotJoints);

    if (!iCartesianSolver->invDyn(q, qdot, qdotdot, fexts, t))
    {
        yCWarning(BCC) << "invDyn() failed, not updating control this iteration";
        return;
    }

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        yCWarning(BCC) << "setRefTorques() failed, not updating control this iteration";
    }
}

// -----------------------------------------------------------------------------

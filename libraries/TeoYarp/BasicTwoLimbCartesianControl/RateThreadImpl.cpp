// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicTwoLimbCartesianControl.hpp"

// ------------------- RateThread Related ------------------------------------

void teo::BasicTwoLimbCartesianControl::run() {

    int catchCurrentState = getCurrentState();
    if (catchCurrentState == VOCAB_CC_MOVS_CONTROLLING)
    {
        double movementTime = yarp::os::Time::now() - movementStartTime;

        if( movementTime > DEFAULT_DURATION )
        {
            this->stopControl();
            return;
        }

        //-- Obtain current joint position
        std::vector<double> currentQA(numRobotJointsA), currentQB(numRobotJointsB);
        if ( ! iEncodersA->getEncoders( currentQA.data() ) )
        {
            CD_WARNING("getEncoders A failed, not updating control this iteration.\n");
            return;
        }
        if ( ! iEncodersB->getEncoders( currentQB.data() ) )
        {
            CD_WARNING("getEncoders B failed, not updating control this iteration.\n");
            return;
        }

        //-- Obtain desired Cartesian position and velocity.
        std::vector<double> desiredX, desiredXdot;
        trajectory->getX(movementTime, desiredX);
        trajectory->getXdot(movementTime, desiredXdot);

        CD_DEBUG_NO_HEADER("[STEP] [%f] x ",movementTime);
        for(int i=0;i<desiredX.size();i++)
            CD_DEBUG_NO_HEADER("%f ",desiredX[i]);
        CD_DEBUG_NO_HEADER("xdot ");
        for(int i=0;i<desiredXdot.size();i++)
            CD_DEBUG_NO_HEADER("%f ",desiredXdot[i]);
        CD_DEBUG_NO_HEADER("\n");

        //-- Apply control law to compute robot Cartesian velocity commands.
        std::vector<double> commandXdotA, commandXdotB;
        iCartesianSolverA->fwdKinError(desiredX,currentQA, commandXdotA);
        for(unsigned int i=0; i<6; i++)
        {
            commandXdotA[i] *= -DEFAULT_GAIN;
            commandXdotA[i] += desiredXdot[i];
        }

        //-- Compute joint velocity commands and send to robot.
        std::vector<double> commandQdotA, commandQdotB;
        if (! iCartesianSolverA->diffInvKin(currentQA,commandXdotA,commandQdotA) )
        {
            CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        }

        CD_DEBUG_NO_HEADER("[STEP] [%f] ",movementTime);
        for(int i=0;i<6;i++)
            CD_DEBUG_NO_HEADER("%f ",commandXdotA[i]);
        CD_DEBUG_NO_HEADER("-> ");
        for(int i=0;i<numRobotJointsA;i++)
            CD_DEBUG_NO_HEADER("%f ",commandQdotA[i]);
        CD_DEBUG_NO_HEADER("[deg/s]\n");

        if( ! iVelocityControlA->velocityMove( commandQdotA.data() ) )
        {
            CD_WARNING("velocityMove failed, not updating control this iteration.\n");
        }

    }

    return;
}

// -----------------------------------------------------------------------------


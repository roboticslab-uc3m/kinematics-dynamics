// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicTwoLimbCartesianControl.hpp"

#include <ColorDebug.h>

// ------------------- RateThread Related ------------------------------------

void roboticslab::BasicTwoLimbCartesianControl::run() {

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
        trajectory->getPosition(movementTime, desiredX);
        trajectory->getVelocity(movementTime, desiredXdot);

        CD_DEBUG_NO_HEADER("[STEP] [%f] x ",movementTime);
        for(int i=0;i<desiredX.size();i++)
            CD_DEBUG_NO_HEADER("%f ",desiredX[i]);
        CD_DEBUG_NO_HEADER("xdot ");
        for(int i=0;i<desiredXdot.size();i++)
            CD_DEBUG_NO_HEADER("%f ",desiredXdot[i]);
        CD_DEBUG_NO_HEADER("\n");


        std::vector<double> desiredXA(desiredX.begin(),desiredX.begin()+6);
        std::vector<double> desiredXB(desiredX.begin()+6,desiredX.end());
        std::vector<double> desiredXdotA(desiredXdot.begin(),desiredXdot.begin()+6);
        std::vector<double> desiredXdotB(desiredXdot.begin()+6,desiredXdot.end());

        //-- Apply control law to compute robot Cartesian velocity commands.
        std::vector<double> commandXdotA, commandXdotB;
        iCartesianSolverA->fwdKinError(desiredXA,currentQA, commandXdotA);
        iCartesianSolverB->fwdKinError(desiredXB,currentQB, commandXdotB);
        for(unsigned int i=0; i<6; i++)
        {
            commandXdotA[i] *= DEFAULT_GAIN * (1000.0 / DEFAULT_MS);
            commandXdotA[i] += desiredXdotA[i];
            commandXdotB[i] *= DEFAULT_GAIN * (1000.0 / DEFAULT_MS);
            commandXdotB[i] += desiredXdotB[i];
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

        std::vector<double> encodersA(numRobotJointsA);
        iEncodersA->getEncoders(encodersA.data());
        std::cout << "rightLeg encoders: " << encodersA << std::endl;
        if (! iCartesianSolverB->diffInvKin(currentQB,commandXdotB,commandQdotB) )
        {
            CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        }

        CD_DEBUG_NO_HEADER("[STEP] [%f] ",movementTime);
        for(int i=0;i<6;i++)
            CD_DEBUG_NO_HEADER("%f ",commandXdotB[i]);
        CD_DEBUG_NO_HEADER("-> ");
        for(int i=0;i<numRobotJointsB;i++)
            CD_DEBUG_NO_HEADER("%f ",commandQdotB[i]);
        CD_DEBUG_NO_HEADER("[deg/s]\n");

        if( ! iVelocityControlB->velocityMove( commandQdotB.data() ) )
        {
            CD_WARNING("velocityMove failed, not updating control this iteration.\n");
        }
        std::vector<double> encodersB(numRobotJointsB);
        iEncodersB->getEncoders(encodersB.data());
        std::cout << "leftLeg encoders: " << encodersB << std::endl;

    }

    return;
}

// -----------------------------------------------------------------------------


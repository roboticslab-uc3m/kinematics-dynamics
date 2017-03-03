// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicTwoLimbCartesianControl.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool teo::BasicTwoLimbCartesianControl::stat(int &state, std::vector<double> &x)
{
    std::vector<double> currentQA(numRobotJointsA), currentQB(numRobotJointsB);
    if ( ! iEncodersA->getEncoders( currentQA.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iEncodersB->getEncoders( currentQB.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    if ( ! iCartesianSolverA->fwdKin(currentQA,x) )
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }
    std::vector<double> xB;
    if ( ! iCartesianSolverB->fwdKin(currentQB,xB) )
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }
    for(int i=0; i<xB.size(); i++)
        x.push_back( xB[i] );

    state = getCurrentState();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicTwoLimbCartesianControl::step()
{
    CD_WARNING("STEP mode still experimental.\n");

    // Initialize trajectory.
    trajectory = new GaitTrajectory();

    //-- Set velocity mode and set state which makes rate thread implement control.
    iVelocityControlA->setVelocityMode();
    iVelocityControlB->setVelocityMode();
    movementStartTime = yarp::os::Time::now();
    setCurrentState( VOCAB_CC_MOVS_CONTROLLING );

    //-- Wait for movement to be done, then delete
    CD_SUCCESS("Waiting\n");
    while( getCurrentState() == VOCAB_CC_MOVS_CONTROLLING )
    {
        printf(".");
        fflush(stdout);
        yarp::os::Time::delay(0.5);
    }
    //trajectory.deleteLine();  //-- Causes segFaults for now

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicTwoLimbCartesianControl::stopControl()
{
    iPositionControlA->setPositionMode();
    iPositionControlA->stop();
    iPositionControlB->setPositionMode();
    iPositionControlB->stop();
    setCurrentState( VOCAB_CC_NOT_CONTROLLING );
    return true;
}

// -----------------------------------------------------------------------------

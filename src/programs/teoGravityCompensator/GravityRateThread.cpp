// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GravityRateThread.hpp"

/************************************************************************/
bool teo::GravityRateThread::threadInit() {

    rightArmEnc->getAxes( &rightArmNumMotors );
    CD_INFO("rightArmNumMotors: %d.\n",rightArmNumMotors);

    rightArmSolver->getNumLinks( &solverRightArmNumLinks );
    CD_INFO("solverRightArmNumLinks: %d.\n",solverRightArmNumLinks);

    if( rightArmNumMotors < solverRightArmNumLinks ) {
        CD_ERROR("rightArmNumMotors < solverRightArmNumLinks !!! (must be >=)\n");
        return false;
    }

    vRightArmAngles.resize( rightArmNumMotors );

    return true;
}

/************************************************************************/
void teo::GravityRateThread::run() {

    rightArmEnc->getEncoders( vRightArmAngles.data() );

    CD_DEBUG("--> ");
    for(int i=0;i<rightArmNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",vRightArmAngles[i]);
    CD_DEBUG_NO_HEADER("\n");

    //rightArmSolver->invDyn()

}

/************************************************************************/

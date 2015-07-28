// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianRateThread.hpp"

/************************************************************************/
bool teo::CartesianRateThread::threadInit() {

    iEncoders->getAxes( &numMotors );
    CD_INFO("numMotors: %d.\n",numMotors);

    solver->getNumLinks( &solverNumLinks );
    CD_INFO("solverNumLinks: %d.\n",solverNumLinks);

    if( numMotors < solverNumLinks ) {
        CD_ERROR("numMotors < solverNumLinks !!! (must be >=)\n");
        return false;
    }

    qReal.resize( numMotors );
    qDotCmd.resize( numMotors );

    iVelocityControl->setVelocityMode();

    return true;
}

/************************************************************************/
void teo::CartesianRateThread::run() {

    iEncoders->getEncoders( qReal.data() );

    CD_DEBUG("<-- ");
    for(int i=0; i<qReal.size(); i++)
        CD_DEBUG_NO_HEADER("%f ",qReal[i]);
    CD_DEBUG_NO_HEADER("[deg]\n");

    solver->fwdKin(qReal, xReal,oReal);

    //xError = xDesired - xReal;
    //xDotCmd = xDotDesired + GAIN * xError * (cmcMs/1000.0);  // GAIN=0 => xCmd = xDotDesired
    //solver->invDiffKin(xDotCmd, qDotCmd);

    CD_INFO("--> ");
    for(int i=0; i<qDotCmd.size(); i++) {
        CD_INFO_NO_HEADER("%f ",qDotCmd[i]);
    }
    CD_INFO_NO_HEADER("[Nm]\n");

    iVelocityControl->velocityMove( qDotCmd.data() );

}

/************************************************************************/

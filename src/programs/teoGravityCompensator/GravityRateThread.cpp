// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GravityRateThread.hpp"

/************************************************************************/
bool teo::GravityRateThread::threadInit() {

    iEncoders->getAxes( &numMotors );
    CD_INFO("numMotorsRA: %d.\n",numMotors);

    solver->getNumLinks( &solverNumLinks );
    CD_INFO("solverNumLinksRA: %d.\n",solverNumLinks);

    if( numMotors < solverNumLinks ) {
        CD_ERROR("numMotorsRA < solverNumLinksRA !!! (must be >=) (RA=rightArm)\n");
        return false;
    }

    q.resize( numMotors );

    iTorqueControl->setTorqueMode();

    return true;
}

/************************************************************************/
void teo::GravityRateThread::run() {

    iEncoders->getEncoders( q.data() );

    CD_DEBUG("<-- ");
    for(int i=0;i<numMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",q[i]);
    CD_DEBUG_NO_HEADER("[deg]\n");

    solver->invDyn(q,t);

    if( numMotors > numMotors )
        t.resize( numMotors );  //-- Extra motors won't care about torques.

    CD_INFO("--> ");
    for(int i=0;i<numMotors;i++) {
        CD_INFO_NO_HEADER("%f ",t[i]);
    }
    CD_INFO_NO_HEADER("[Nm]\n");

    //--tRA[0] = 0.0;  //-- Release... let's do this!
    iTorqueControl->setRefTorques( t.data() );

}

/************************************************************************/

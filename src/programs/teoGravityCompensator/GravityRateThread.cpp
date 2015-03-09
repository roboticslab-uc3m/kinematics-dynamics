// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GravityRateThread.hpp"

/************************************************************************/
bool teo::GravityRateThread::threadInit() {

    iEncodersRA->getAxes( &numMotorsRA );
    CD_INFO("numMotorsRA: %d.\n",numMotorsRA);

    solverRA->getNumLinks( &solverNumLinksRA );
    CD_INFO("solverNumLinksRA: %d.\n",solverNumLinksRA);

    if( numMotorsRA < solverNumLinksRA ) {
        CD_ERROR("numMotorsRA < solverNumLinksRA !!! (must be >=) (RA=rightArm)\n");
        return false;
    }

    qRA.resize( numMotorsRA );

    iTorqueControlRA->setTorqueMode();

    return true;
}

/************************************************************************/
void teo::GravityRateThread::run() {

    iEncodersRA->getEncoders( qRA.data() );

    CD_DEBUG("<-- ");
    for(int i=0;i<numMotorsRA;i++)
        CD_DEBUG_NO_HEADER("%f ",qRA[i]);
    CD_DEBUG_NO_HEADER("\n");

    solverRA->invDyn(qRA,tRA);

    if( numMotorsRA > numMotorsRA )
        tRA.resize( numMotorsRA );  //-- Extra motors won't care about torques.

    CD_INFO("--> ");
    for(int i=0;i<numMotorsRA;i++) {
        CD_INFO_NO_HEADER("%f ",tRA[i]);
    }
    CD_INFO_NO_HEADER("\n");

    tRA[0] = 0.0;
    iTorqueControlRA->setRefTorques( tRA.data() );

}

/************************************************************************/

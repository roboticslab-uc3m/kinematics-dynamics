// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GravityRateThread.hpp"

/************************************************************************/
bool teo::GravityRateThread::threadInit() {

    iEncodersRA->getAxes( &numMotorsRA );
    CD_INFO("rightArmNumMotors: %d.\n",numMotorsRA);

    solverRA->getNumLinks( &solverNumLinksRA );
    CD_INFO("solverRightArmNumLinks: %d.\n",solverNumLinksRA);

    if( numMotorsRA < solverNumLinksRA ) {
        CD_ERROR("rightArmNumMotors < solverRightArmNumLinks !!! (must be >=)\n");
        return false;
    }

    qRA.resize( numMotorsRA );

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

    CD_INFO("--> ");
    for(int i=0;i<solverNumLinksRA;i++)
        CD_DEBUG_NO_HEADER("%f ",tRA[i]);
    CD_DEBUG_NO_HEADER("\n");

}

/************************************************************************/

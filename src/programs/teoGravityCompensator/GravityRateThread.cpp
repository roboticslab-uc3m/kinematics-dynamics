// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GravityRateThread.hpp"

/************************************************************************/
void GravityRateThread::run() {

    //-- Make room!!
    std::vector< double > vRightArmAngles( rightArmNumMotors );
    double* rightArmAngles = vRightArmAngles.data();

    rightArmEnc->getEncoders( rightArmAngles );

    CD_DEBUG("--> ");
    for(int i=0;i<rightArmNumMotors;i++)
        CD_DEBUG_NO_HEADER("%f ",rightArmAngles[i]);
    CD_DEBUG_NO_HEADER("\n");

}

/************************************************************************/

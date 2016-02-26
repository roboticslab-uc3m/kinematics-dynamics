// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ------------------- RateThread Related ------------------------------------

bool teo::FakeControlboard::threadInit() {
    printf("[FakeControlboard] success: threadInit()\n");
    lastTime = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------

void teo::FakeControlboard::run() {
    for(unsigned int motor=0;motor<axes;motor++){
        setEncRaw(motor, getEncRaw(motor)+(velRaw[motor])*(yarp::os::Time::now()-lastTime));
        if((jointStatus[motor]==1)||(jointStatus[motor]==2)||(jointStatus[motor]==3)) {  // if set to move...
            if ((getEncExposed(motor) > maxLimit[motor])  && (velRaw[motor]>0)) {  // SW max JL
                stop(motor);  // puts jointStatus[motor]=0;
                fprintf(stderr,"[FakeControlboard] warning: Moving joint q%d at configured max joint limit, stopping.\n",motor+1);
            } else if ((getEncExposed(motor) < minLimit[motor]) && (velRaw[motor]<0)) {  // SW min JL
                stop(motor);  // puts jointStatus[motor]=0;
                fprintf(stderr,"[FakeControlboard] warning: Moving joint q%d at configured min joint limit, stopping.\n",motor+1);
            } else if((jointStatus[motor]==1)||(jointStatus[motor]==2)) {  // check if target reached in pos or rel
                if ( (velRaw[motor] > 0) &&  // moving positive...
                    (getEncExposed(motor) > (targetExposed[motor]-jointTol[motor])) ) {
                    stop(motor);  // puts jointStatus[motor]=0;
                    printf("[FakeControlboard] Joint q%d reached target.\n",motor+1);
                } else if ( (velRaw[motor] < 0) &&  // moving negative...
                    (getEncExposed(motor) < (targetExposed[motor]+jointTol[motor])) ) {
                    stop(motor);  // puts jointStatus[motor]=0;
                    printf("[FakeControlboard] Joint q%d reached target.\n",motor+1);
                }
            }
        }
    }
    lastTime = yarp::os::Time::now();

}

// -----------------------------------------------------------------------------


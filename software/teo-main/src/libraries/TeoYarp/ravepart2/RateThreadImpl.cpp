// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RavePart2.hpp"

// ------------------- RateThread Related ------------------------------------

bool teo::RavePart2::threadInit() {
    printf("[RavePart2] success: threadInit()\n");
    lastTime = Time::now();
    return true;
}

// -----------------------------------------------------------------------------

void teo::RavePart2::run() {
    for(unsigned int motor=0;motor<axes;motor++){
        setEncRaw(motor, getEncRaw(motor)+(velRaw[motor])*(Time::now()-lastTime));
        if((jointStatus[motor]==1)||(jointStatus[motor]==2)||(jointStatus[motor]==3)) {  // if set to move...
            if ((getEncExposed(motor) > maxLimit[motor])  && (velRaw[motor]>0)) {  // SW max JL
                stop(motor);  // puts jointStatus[motor]=0;
                fprintf(stderr,"[RavePart2] warning: Moving joint q%d at configured max joint limit, stopping.\n",motor+1);
            } else if ((getEncExposed(motor) < minLimit[motor]) && (velRaw[motor]<0)) {  // SW min JL
                stop(motor);  // puts jointStatus[motor]=0;
                fprintf(stderr,"[RavePart2] warning: Moving joint q%d at configured min joint limit, stopping.\n",motor+1);
            } else if((jointStatus[motor]==1)||(jointStatus[motor]==2)) {  // check if target reached in pos or rel
                if ( (velRaw[motor] > 0) &&  // moving positive...
                    (getEncExposed(motor) > (targetExposed[motor]-jointTol[motor])) ) {
                    stop(motor);  // puts jointStatus[motor]=0;
                    printf("[RavePart2] Joint q%d reached target.\n",motor+1);
                } else if ( (velRaw[motor] < 0) &&  // moving negative...
                    (getEncExposed(motor) < (targetExposed[motor]+jointTol[motor])) ) {
                    stop(motor);  // puts jointStatus[motor]=0;
                    printf("[RavePart2] Joint q%d reached target.\n",motor+1);
                }
            }
        }
    }
    lastTime = Time::now();

}

// -----------------------------------------------------------------------------


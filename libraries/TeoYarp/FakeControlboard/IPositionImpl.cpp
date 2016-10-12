// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ------------------- IPositionControl Related --------------------------------

bool teo::FakeControlboard::getAxes(int *ax) {
    *ax = axes;
    CD_INFO("Reporting %d axes are present\n", *ax);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setPositionMode() {
    CD_INFO("\n");
    if (modePosVel==0) return true;  // Simply return true if we were already in pos mode.
    // Do anything additional before setting flag to pos...
    if(!stop()) {
        fprintf(stderr,"[FakeControlboard] warning: setPositionMode() return false; failed to stop\n");
        return false;
    }
    modePosVel = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::positionMove(int j, double ref) {  // encExposed = ref;
    if ((unsigned int)j>axes) {
        fprintf(stderr,"[FakeControlboard] error: axis index more than axes.\n");
        return false;
    }
    if(modePosVel!=0) {  // Check if we are in position mode.
        fprintf(stderr,"[FakeControlboard] warning: will not positionMove as not in positionMode\n");
        return false;
    }
    printf("[FakeControlboard] positionMove(%d,%f) f[begin]\n",j,ref);
    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    targetExposed[j] = ref;
    if (fabs(targetExposed[j]-getEncExposed(j))<jointTol[j]) {
        stop(j);  // puts jointStatus[j]=0;
        printf("[FakeControlboard] Joint q%d reached target.\n",j+1);
        return true;
    } else if ( ref > getEncExposed(j) ) {
        //if(!velocityMove(j, refSpeed[j])) return false;
        velRaw[j] = (refSpeed[j] * velRawExposed[j]);
    } else {
        //if(!velocityMove(j, -refSpeed[j])) return false;
        velRaw[j] = -(refSpeed[j] * velRawExposed[j]);
    }
    jointStatus[j] = 1;
    printf("[FakeControlboard] positionMove(%d,%f) f[end]\n",j,ref);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::positionMove(const double *refs) {  // encExposed = refs;
    if(modePosVel!=0) {  // Check if we are in position mode.
        fprintf(stderr,"[FakeControlboard] error: Will not positionMove as not in positionMode\n");
        return false;
    }
    printf("[FakeControlboard] positionMove() f[begin]\n");
    // Find out the maximum time to move
    double max_time = 0;
    for(unsigned int motor=0;motor<axes;motor++) {
        printf("[FakeControlboard] dist[%d]: %f\n",motor,fabs(refs[motor]-getEncExposed(motor)));
        printf("[FakeControlboard] refSpeed[%d]: %f\n",motor,refSpeed[motor]);
        if (fabs((refs[motor]-getEncExposed(motor))/refSpeed[motor])>max_time) {
            max_time = fabs((refs[motor]-getEncExposed(motor))/refSpeed[motor]);
            printf("[FakeControlboard] -->candidate: %f\n",max_time);
        }
    }
    printf("[FakeControlboard] max_time[final]: %f\n",max_time);
    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    for(unsigned int motor=0;motor<axes;motor++) {
        targetExposed[motor]=refs[motor];
        velRaw[motor] = ((refs[motor]-getEncExposed(motor))/max_time)*velRawExposed[motor];
        if(velRaw[motor] != velRaw[motor]) velRaw[motor] = 0;  // protect against NaN
        printf("[FakeControlboard] velRaw[%d]: %f\n",motor,velRaw[motor]);
        jointStatus[motor]=1;
        if (fabs(targetExposed[motor]-getEncExposed(motor))<jointTol[motor]) {
            stop(motor);  // puts jointStatus[motor]=0;
            printf("[FakeControlboard] Joint q%d reached target.\n",motor+1);
        }
    }
    printf("[FakeControlboard] positionMove() f[end]\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::relativeMove(int j, double delta) {
    if ((unsigned int)j>axes) return false;
    if(modePosVel!=0) {  // Check if we are in position mode.
        printf("[fail] FakeControlboard will not relativeMove as not in positionMode\n");
        return false;
    }
    printf("[FakeControlboard] relativeMove(%d,%f) f[begin]\n",j,delta);
    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    targetExposed[j]=getEncExposed(j)+delta;
    if (fabs(targetExposed[j]-getEncExposed(j))<jointTol[j]) {
        stop(j);  // puts jointStatus[j]=0;
        printf("[FakeControlboard] Joint q%d already at target.\n",j+1);
        return true;
    } else if ( targetExposed[j] > getEncExposed(j) ) {
        // if(!velocityMove(j, refSpeed[j])) return false;
        velRaw[j] = (refSpeed[j] * velRawExposed[j]);
    } else {
        // if(!velocityMove(j, -refSpeed[j])) return false;
        velRaw[j] = -(refSpeed[j] * velRawExposed[j]);
    }
    jointStatus[j]=2;
    printf("[FakeControlboard] relativeMove(%d,%f) f[end]\n",j,delta);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::relativeMove(const double *deltas) {  // encExposed = deltas + encExposed
    if(modePosVel!=0) {  // Check if we are in position mode.
        fprintf(stderr,"[FakeControlboard] warning: will not relativeMove as not in positionMode\n");
        return false;
    }
    printf("[FakeControlboard] relativeMove() f[begin]\n");
    // Find out the maximum angle to move
    double max_dist = 0;
    double time_max_dist = 0;
    for(unsigned int motor=0;motor<axes;motor++)
        if (fabs(deltas[motor])>max_dist) {
            max_dist = fabs(deltas[motor]);
            time_max_dist = max_dist/refSpeed[motor];  // the max_dist motor will be at refSpeed
        }
    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    for(unsigned int motor=0; motor<axes; motor++) {
      targetExposed[motor]=getEncExposed(motor)+deltas[motor];
      velRaw[motor] = ((deltas[motor])/time_max_dist)*velRawExposed[motor];
      printf("velRaw[%d]: %f\n",motor,velRaw[motor]);
      jointStatus[motor]=2;
    }
    printf("[FakeControlboard] relativeMove() f[end]\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::checkMotionDone(int j, bool *flag) {
    if ((unsigned int)j>axes) return false;
    bool done = true;
    if (jointStatus[j]>0) done=false;
    *flag = done;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::checkMotionDone(bool *flag) {
    bool done = true;
    for (unsigned int i=0; i<axes; i++) {
        if (jointStatus[i]>0) done = false;
    }
    *flag = done;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setRefSpeed(int j, double sp) {
    if ((unsigned int)j>axes) return false;
    refSpeed[j]=sp;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setRefSpeeds(const double *spds) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= setRefSpeed(i,spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setRefAcceleration(int j, double acc) {
    if ((unsigned int)j>axes) return false;
    refAcc[j]=acc;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setRefAccelerations(const double *accs) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= setRefAcceleration(i,accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getRefSpeed(int j, double *ref) {
    if ((unsigned int)j>axes) return false;
    *ref=refSpeed[j];
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getRefSpeeds(double *spds) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= getRefSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getRefAcceleration(int j, double *acc) {
    if ((unsigned int)j>axes) return false;
    *acc=refAcc[j];
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getRefAccelerations(double *accs) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= getRefAcceleration(i,&accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::stop(int j) {
    if ((unsigned int)j>axes) return false;
    printf("[FakeControlboard] stop(%d)\n",j);
    velRaw[j]=0.0;
    jointStatus[j]=0;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::stop() {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= stop(i);
    return ok;
}

// -----------------------------------------------------------------------------


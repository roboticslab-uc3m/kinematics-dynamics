// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboardOR.hpp"

// ------------------- IPositionControl Related --------------------------------

bool teo::FakeControlboardOR::getAxes(int *ax) {
    *ax = axes;
    CD_INFO("Reporting %d axes are present\n", *ax);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setPositionMode() {
    CD_INFO("\n");
    if (modePosVel==0) return true;  // Simply return true if we were already in pos mode.
    // Do anything additional before setting flag to pos...
    if(!stop()) {
        fprintf(stderr,"[FakeControlboardOR] warning: setPositionMode() return false; failed to stop\n");
        return false;
    }
    modePosVel = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::positionMove(int j, double ref) {  // encExposed = ref;
    if ((unsigned int)j>axes) {
        fprintf(stderr,"[FakeControlboardOR] error: axis index more than axes.\n");
        return false;
    }
    if(modePosVel!=0) {  // Check if we are in position mode.
        fprintf(stderr,"[FakeControlboardOR] warning: will not positionMove as not in positionMode\n");
        return false;
    }

    /*CD_DEBUG("penv %p\n",penv);
    CD_DEBUG("probot %p\n",probot);
    boost::shared_ptr<OpenRAVE::EnvironmentBase> bpenv(penv);
    boost::shared_ptr<OpenRAVE::RobotBase> bprobot(probot);
    CD_DEBUG("penv %p\n",bpenv.get());
    CD_DEBUG("probot %p\n",bprobot.get());
    if(bpenv->CheckSelfCollision(bprobot)) {  // Check if we collide.
        fprintf(stderr,"[FakeControlboardOR] warning: collide\n");
        return false;
    }*/

    OpenRAVE::EnvironmentBasePtr bpenv(penv);



    printf("[FakeControlboardOR] positionMove(%d,%f) f[begin]\n",j,ref);
    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    targetExposed[j] = ref;
    if (fabs(targetExposed[j]-getEncExposed(j))<jointTol[j]) {
        stop(j);  // puts jointStatus[j]=0;
        printf("[FakeControlboardOR] Joint q%d reached target.\n",j+1);
        return true;
    } else if ( ref > getEncExposed(j) ) {
        //if(!velocityMove(j, refSpeed[j])) return false;
        velRaw[j] = (refSpeed[j] * velRawExposed[j]);
    } else {
        //if(!velocityMove(j, -refSpeed[j])) return false;
        velRaw[j] = -(refSpeed[j] * velRawExposed[j]);
    }
    jointStatus[j] = 1;
    printf("[FakeControlboardOR] positionMove(%d,%f) f[end]\n",j,ref);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::positionMove(const double *refs) {  // encExposed = refs;
    if(modePosVel!=0) {  // Check if we are in position mode.
        fprintf(stderr,"[FakeControlboardOR] error: Will not positionMove as not in positionMode\n");
        return false;
    }
    printf("[FakeControlboardOR] positionMove() f[begin]\n");
    // Find out the maximum time to move
    double max_time = 0;
    for(unsigned int motor=0;motor<axes;motor++) {
        printf("[FakeControlboardOR] dist[%d]: %f\n",motor,fabs(refs[motor]-getEncExposed(motor)));
        printf("[FakeControlboardOR] refSpeed[%d]: %f\n",motor,refSpeed[motor]);
        if (fabs((refs[motor]-getEncExposed(motor))/refSpeed[motor])>max_time) {
            max_time = fabs((refs[motor]-getEncExposed(motor))/refSpeed[motor]);
            printf("[FakeControlboardOR] -->candidate: %f\n",max_time);
        }
    }
    printf("[FakeControlboardOR] max_time[final]: %f\n",max_time);
    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    for(unsigned int motor=0;motor<axes;motor++) {
        targetExposed[motor]=refs[motor];
        velRaw[motor] = ((refs[motor]-getEncExposed(motor))/max_time)*velRawExposed[motor];
        if(velRaw[motor] != velRaw[motor]) velRaw[motor] = 0;  // protect against NaN
        printf("[FakeControlboardOR] velRaw[%d]: %f\n",motor,velRaw[motor]);
        jointStatus[motor]=1;
        if (fabs(targetExposed[motor]-getEncExposed(motor))<jointTol[motor]) {
            stop(motor);  // puts jointStatus[motor]=0;
            printf("[FakeControlboardOR] Joint q%d reached target.\n",motor+1);
        }
    }
    printf("[FakeControlboardOR] positionMove() f[end]\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::relativeMove(int j, double delta) {
    if ((unsigned int)j>axes) return false;
    if(modePosVel!=0) {  // Check if we are in position mode.
        printf("[fail] FakeControlboardOR will not relativeMove as not in positionMode\n");
        return false;
    }
    printf("[FakeControlboardOR] relativeMove(%d,%f) f[begin]\n",j,delta);
    // Set all the private parameters of the Rave class that correspond to this kind of movement!
    targetExposed[j]=getEncExposed(j)+delta;
    if (fabs(targetExposed[j]-getEncExposed(j))<jointTol[j]) {
        stop(j);  // puts jointStatus[j]=0;
        printf("[FakeControlboardOR] Joint q%d already at target.\n",j+1);
        return true;
    } else if ( targetExposed[j] > getEncExposed(j) ) {
        // if(!velocityMove(j, refSpeed[j])) return false;
        velRaw[j] = (refSpeed[j] * velRawExposed[j]);
    } else {
        // if(!velocityMove(j, -refSpeed[j])) return false;
        velRaw[j] = -(refSpeed[j] * velRawExposed[j]);
    }
    jointStatus[j]=2;
    printf("[FakeControlboardOR] relativeMove(%d,%f) f[end]\n",j,delta);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::relativeMove(const double *deltas) {  // encExposed = deltas + encExposed
    if(modePosVel!=0) {  // Check if we are in position mode.
        fprintf(stderr,"[FakeControlboardOR] warning: will not relativeMove as not in positionMode\n");
        return false;
    }
    printf("[FakeControlboardOR] relativeMove() f[begin]\n");
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
    printf("[FakeControlboardOR] relativeMove() f[end]\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::checkMotionDone(int j, bool *flag) {
    if ((unsigned int)j>axes) return false;
    bool done = true;
    if (jointStatus[j]>0) done=false;
    *flag = done;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::checkMotionDone(bool *flag) {
    bool done = true;
    for (unsigned int i=0; i<axes; i++) {
        if (jointStatus[i]>0) done = false;
    }
    *flag = done;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setRefSpeed(int j, double sp) {
    if ((unsigned int)j>axes) return false;
    refSpeed[j]=sp;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setRefSpeeds(const double *spds) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= setRefSpeed(i,spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setRefAcceleration(int j, double acc) {
    if ((unsigned int)j>axes) return false;
    refAcc[j]=acc;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setRefAccelerations(const double *accs) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= setRefAcceleration(i,accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefSpeed(int j, double *ref) {
    if ((unsigned int)j>axes) return false;
    *ref=refSpeed[j];
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefSpeeds(double *spds) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= getRefSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefAcceleration(int j, double *acc) {
    if ((unsigned int)j>axes) return false;
    *acc=refAcc[j];
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefAccelerations(double *accs) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= getRefAcceleration(i,&accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::stop(int j) {
    if ((unsigned int)j>axes) return false;
    printf("[FakeControlboardOR] stop(%d)\n",j);
    velRaw[j]=0.0;
    jointStatus[j]=0;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::stop() {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= stop(i);
    return ok;
}

// -----------------------------------------------------------------------------


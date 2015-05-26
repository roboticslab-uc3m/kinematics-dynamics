// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ------------------ IVelocity Related ----------------------------------------

bool teo::FakeControlboard::setVelocityMode() {
    CD_INFO("\n");
    if (modePosVel==1) return true;  // Simply return true if we were already in vel mode.
    // Do anything additional before setting flag to vel...
    modePosVel = 1;  // Set flag to vel.
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::velocityMove(int j, double sp) {  // velExposed = sp;
    if ((unsigned int)j>axes) return false;
    if(modePosVel!=1) {  // Check if we are in velocity mode.
        fprintf(stderr,"[FakeControlboard] fail: FakeControlboard will not velocityMove as not in velocityMode\n");
        return false;
    }
    velRaw[j] = (sp * velRawExposed[j]);
    jointStatus[j]=3;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::velocityMove(const double *sp) {
    printf("[FakeControlboard] Vel:");
    for (unsigned int i=0; i<axes; i++) printf(" %+.6f",velRaw[i]);
    printf("\n");
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= velocityMove(i,sp[i]);
    return ok;
}

// ----------------------------------------------------------------------------


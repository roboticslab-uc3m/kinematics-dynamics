// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboardOR.hpp"

// ------------------ IEncoders Related -----------------------------------------

bool teo::FakeControlboardOR::resetEncoder(int j) {
    if ((unsigned int)j>axes) return false;
    return setEncoder(j,0.0);
  }

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::resetEncoders() {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= resetEncoder(i);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setEncoder(int j, double val) {  // encExposed = val;
    setEncRaw(j, val * encRawExposed[j]);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setEncoders(const double *vals) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= setEncoder(i,vals[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getEncoder(int j, double *v) {
    *v = getEncExposed(j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getEncoders(double *encs) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= getEncoder(i,&encs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getEncoderSpeed(int j, double *sp) {
    // Make it easy, give the current reference speed.
    *sp = velRaw[j] / velRawExposed[j];  // begins to look like we should use semaphores.
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getEncoderSpeeds(double *spds) {
    bool ok = true;
    for(unsigned int i=0;i<axes;i++)
        ok &= getEncoderSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getEncoderAcceleration(int j, double *spds) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getEncoderAccelerations(double *accs) {
    return false;
}

// -----------------------------------------------------------------------------


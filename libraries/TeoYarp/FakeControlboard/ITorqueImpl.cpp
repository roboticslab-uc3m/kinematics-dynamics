// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RavePart.hpp"

// ------------------- IForceControl Related ------------------------------------


bool teo::RavePart::setTorqueMode() {
    CD_INFO("setTorqueMode()\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getRefTorques(double *t){
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getRefTorque(int j, double *t) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setRefTorques(const double *t) {
    return true;
}


// -----------------------------------------------------------------------------

bool teo::RavePart::setRefTorque(int j, double t) {
    CD_INFO("joint: %d, refTorque: %f.\n",j,t);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getBemfParam(int j, double *bemf) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setBemfParam(int j, double bemf) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setTorquePid(int j, const Pid &pid) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorque(int j, double *t) {
    //CD_INFO("joint: %d.\n",j);  //-- Way too verbose
    *t = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorques(double *t) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorqueRange(int j, double *min, double *max) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorqueRanges(double *min, double *max) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setTorquePids(const Pid *pids) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setTorqueErrorLimit(int j, double limit) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setTorqueErrorLimits(const double *limits) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorqueError(int j, double *err) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorqueErrors(double *errs) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorquePidOutput(int j, double *out) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorquePidOutputs(double *outs) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorquePid(int j, Pid *pid) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorquePids(Pid *pids){
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorqueErrorLimit(int j, double *limit) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getTorqueErrorLimits(double *limits) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::resetTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::disableTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::enableTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setTorqueOffset(int j, double v) {
    return true;
}

// -----------------------------------------------------------------------------

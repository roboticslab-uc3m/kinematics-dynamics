// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RavePart2.hpp"

// ------------------- IForceControl Related ------------------------------------


bool teo::RavePart2::setTorqueMode() {
    CD_INFO("setPositionMode()\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getRefTorques(double *t){
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getRefTorque(int j, double *t) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setRefTorques(const double *t) {
    return true;
}


// -----------------------------------------------------------------------------

bool teo::RavePart2::setRefTorque(int j, double t) {
    CD_INFO("joint: %d, refTorque: %f.\n",j,t);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getBemfParam(int j, double *bemf) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setBemfParam(int j, double bemf) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setTorquePid(int j, const Pid &pid) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorque(int j, double *t) {
    CD_INFO("joint: %d.\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorques(double *t) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorqueRange(int j, double *min, double *max) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorqueRanges(double *min, double *max) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setTorquePids(const Pid *pids) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setTorqueErrorLimit(int j, double limit) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setTorqueErrorLimits(const double *limits) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorqueError(int j, double *err) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorqueErrors(double *errs) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorquePidOutput(int j, double *out) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorquePidOutputs(double *outs) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorquePid(int j, Pid *pid) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorquePids(Pid *pids){
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorqueErrorLimit(int j, double *limit) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getTorqueErrorLimits(double *limits) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::resetTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::disableTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::enableTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setTorqueOffset(int j, double v) {
    return true;
}

// -----------------------------------------------------------------------------

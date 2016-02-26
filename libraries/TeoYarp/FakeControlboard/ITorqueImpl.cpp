// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ------------------- IForceControl Related ------------------------------------


bool teo::FakeControlboard::setTorqueMode() {
    CD_INFO("setTorqueMode()\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getRefTorques(double *t){
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getRefTorque(int j, double *t) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setRefTorques(const double *t) {
    return true;
}


// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setRefTorque(int j, double t) {
    CD_INFO("joint: %d, refTorque: %f.\n",j,t);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getBemfParam(int j, double *bemf) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setBemfParam(int j, double bemf) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setTorquePid(int j, const yarp::dev::Pid &pid) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorque(int j, double *t) {
    //CD_INFO("joint: %d.\n",j);  //-- Way too verbose
    *t = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorques(double *t) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorqueRange(int j, double *min, double *max) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorqueRanges(double *min, double *max) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setTorquePids(const yarp::dev::Pid *pids) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setTorqueErrorLimit(int j, double limit) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setTorqueErrorLimits(const double *limits) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorqueError(int j, double *err) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorqueErrors(double *errs) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorquePidOutput(int j, double *out) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorquePidOutputs(double *outs) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorquePid(int j, yarp::dev::Pid *pid) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorquePids(yarp::dev::Pid *pids){
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorqueErrorLimit(int j, double *limit) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTorqueErrorLimits(double *limits) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::resetTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::disableTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::enableTorquePid(int j) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setTorqueOffset(int j, double v) {
    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RavePart.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool teo::RavePart::setPositionMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setVelocityMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setTorqueMode(int j)  {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setImpedancePositionMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setImpedanceVelocityMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::setOpenLoopMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getControlMode(int j, int *mode) {
    return true;
}

// -----------------------------------------------------------------------------


bool teo::RavePart::getControlModes(int *modes) {
    return true;
}

// -----------------------------------------------------------------------------

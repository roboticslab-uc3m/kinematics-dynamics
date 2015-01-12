// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RavePart2.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool teo::RavePart2::setPositionMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setVelocityMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setTorqueMode(int j)  {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setImpedancePositionMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setImpedanceVelocityMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::setOpenLoopMode(int j) {
    CD_INFO("(%d)\n",j);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getControlMode(int j, int *mode) {
    return true;
}

// -----------------------------------------------------------------------------


bool teo::RavePart2::getControlModes(int *modes) {
    return true;
}

// -----------------------------------------------------------------------------

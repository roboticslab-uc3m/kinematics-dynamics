// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboardOR.hpp"

// ------------------- IControlLimits2 Related ------------------------------------

bool teo::FakeControlboardOR::setVelLimits(int axis, double min, double max) {
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getVelLimits(int axis, double *min, double *max) {
    CD_DEBUG("\n");
    // yarpmotorgui's defaults (partitem.cpp)
    *min = 0;
    *max = 100;
    return true;
}

// -----------------------------------------------------------------------------


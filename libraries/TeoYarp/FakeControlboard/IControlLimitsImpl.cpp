// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool teo::FakeControlboard::setLimits(int axis, double min, double max) {
    if(axis>=int(axes)) return false;
    minLimit[axis] = min;
    maxLimit[axis] = max;
    printf("[FakeControlboard] Range of axis %d set to: %f to %f\n",axis,min,max);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getLimits(int axis, double *min, double *max) {
    if(axis>=int(axes)) return false;
    *min = minLimit[axis];
    *max = maxLimit[axis];
    printf("[FakeControlboard] Range of axis %d read: %f to %f.\n",axis,*min,*max);
    return true;
}

// -----------------------------------------------------------------------------


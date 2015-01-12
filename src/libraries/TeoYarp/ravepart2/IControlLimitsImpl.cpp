// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RavePart2.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool teo::RavePart2::setLimits(int axis, double min, double max) {
    if(axis>=int(axes)) return false;
    minLimit[axis] = min;
    maxLimit[axis] = max;
    printf("[RavePart2] Range of axis %d set to: %f to %f\n",axis,min,max);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart2::getLimits(int axis, double *min, double *max) {
    if(axis>=int(axes)) return false;
    *min = minLimit[axis];
    *max = maxLimit[axis];
    printf("[RavePart2] Range of axis %d read: %f to %f.\n",axis,*min,*max);
    return true;
}

// -----------------------------------------------------------------------------


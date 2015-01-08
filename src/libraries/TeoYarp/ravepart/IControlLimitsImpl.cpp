// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RavePart.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool teo::RavePart::setLimits(int axis, double min, double max) {
    if(axis>=int(axes)) return false;
    minLimit[axis] = min;
    maxLimit[axis] = max;
    printf("[RavePart] Range of axis %d set to: %f to %f\n",axis,min,max);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::RavePart::getLimits(int axis, double *min, double *max) {
    if(axis>=int(axes)) return false;
    *min = minLimit[axis];
    *max = maxLimit[axis];
    printf("[RavePart] Range of axis %d read: %f to %f.\n",axis,*min,*max);
    return true;
}

// -----------------------------------------------------------------------------


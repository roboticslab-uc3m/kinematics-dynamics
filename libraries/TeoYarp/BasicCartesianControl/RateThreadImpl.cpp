// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- RateThread Related ------------------------------------

void teo::BasicCartesianControl::run() {

    if (currentState == VOCAB_CC_MOVL_CONTROLLING)
    {
        CD_DEBUG("MOVEL_CONTROLLING\n");
    }
    return;
}

// -----------------------------------------------------------------------------


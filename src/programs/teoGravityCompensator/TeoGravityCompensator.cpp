// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoGravityCompensator.hpp"

/************************************************************************/
TeoGravityCompensator::TeoGravityCompensator() { }

/************************************************************************/
bool TeoGravityCompensator::configure(ResourceFinder &rf) {

    return true;
}

/************************************************************************/
bool TeoGravityCompensator::updateModule() {
    // printf("Alive\n");
    return true;
}

/************************************************************************/
bool TeoGravityCompensator::interruptModule() {
    return true;
}

/************************************************************************/


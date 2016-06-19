// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::BasicCartesianControl::open(yarp::os::Searchable& config) {

    //numLinks = config.check("numLinks",yarp::os::Value(DEFAULT_NUM_LINKS),"chain number of segments").asInt();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::close() {

    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlClientROS2::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(CCC) << "Failed to parse parameters";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::close()
{
    return true;
}

// -----------------------------------------------------------------------------

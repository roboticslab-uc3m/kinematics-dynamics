// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AmorCartesianControl::open(yarp::os::Searchable& config)
{
    CD_DEBUG("AmorCartesianControl config: %s.\n", config.toString().c_str());

    yarp::os::Property robotOptions;
    robotOptions.fromString(config.toString());
    robotOptions.put("device", "AmorControlboard");

    robotDevice.open(robotOptions);

    if (!robotDevice.isValid())
    {
        CD_ERROR("Robot device not valid.\n");
        return false;
    }

    if (!robotDevice.view(iEncoders))
    {
        CD_ERROR("Could not view iEncoders.\n");
        return false;
    }

    if (!robotDevice.view(iPositionControl))
    {
        CD_ERROR("Could not view iPositionControl.\n");
        return false;
    }

    if (!robotDevice.view(iVelocityControl))
    {
        CD_ERROR("Could not view iVelocityControl.\n");
        return false;
    }

    if (!robotDevice.view(iControlLimits))
    {
        CD_ERROR("Could not view iControlLimits.\n");
        return false;
    }

    if (!robotDevice.view(iTorqueControl))
    {
        CD_ERROR("Could not view iTorqueControl.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::close()
{
    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AmorCartesianControl::open(yarp::os::Searchable& config)
{
    CD_DEBUG("AmorCartesianControl config: %s.\n", config.toString().c_str());

    std::string kinematicsFile = config.check("kinematics", yarp::os::Value(""),
            "AMOR kinematics description").asString();

    yarp::os::Property cartesianDeviceOptions;

    if (!cartesianDeviceOptions.fromConfigFile(kinematicsFile))
    {
        CD_ERROR("Cannot read from --kinematics \"%s\".\n", kinematicsFile.c_str());
        return false;
    }

    cartesianDeviceOptions.put("device", "KdlSolver");

    if (!cartesianDevice.open(cartesianDeviceOptions))
    {
        CD_ERROR("Solver device not valid.\n");
        return false;
    }

    if (!cartesianDevice.view(iCartesianSolver))
    {
        CD_ERROR("Could not view iCartesianSolver.\n");
        close();
        return false;
    }

    yarp::os::Value vHandle = config.find("handle");

    if (vHandle.isNull())
    {
        CD_INFO("Creating own AMOR handle.\n");

        std::string canLibrary = config.check("canLibrary", yarp::os::Value(DEFAULT_CAN_LIBRARY),
                "CAN plugin library").asString();
        int canPort = config.check("canPort", yarp::os::Value(DEFAULT_CAN_PORT),
                "CAN port number").asInt();

        ownsHandle = true;
        handle = amor_connect(const_cast<char *>(canLibrary.c_str()), canPort);
    }
    else
    {
        CD_INFO("Using external AMOR handle.\n");
        ownsHandle = false;
        handle = *((AMOR_HANDLE *)vHandle.asBlob());
    }

    if (handle == AMOR_INVALID_HANDLE)
    {
        CD_ERROR("Could not get AMOR handle (%s)\n", amor_error());
        close();
        return false;
    }

    CD_SUCCESS("Acquired AMOR handle!\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::close()
{
    CD_INFO("Closing AmorCartesianControl...\n");

    if (handle != AMOR_INVALID_HANDLE)
    {
        amor_emergency_stop(handle);
    }

    if (ownsHandle)
    {
        amor_release(handle);
    }

    handle = AMOR_INVALID_HANDLE;

    return cartesianDevice.close();
}

// -----------------------------------------------------------------------------

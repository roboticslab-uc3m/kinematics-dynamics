// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FtCompensation.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_SENSOR_LOCAL_PORT = "/ftCompensation";

bool FtCompensation::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(FTC) << "Config:" << rf.toString();

    if (!rf.check("sensorName", "remote FT sensor name to connect to via MAS client"))
    {
        yCError(FTC) << "Missing parameter: sensorName";
        return false;
    }

    auto sensorName = rf.find("sensorName").asString();

    if (!rf.check("sensorRemote", "remote FT sensor port to connect to via MAS client"))
    {
        yCError(FTC) << "Missing parameter: sensorRemote";
        return false;
    }

    auto sensorRemote = rf.find("sensorRemote").asString();
    auto sensorLocal = rf.check("sensorLocal", yarp::os::Value(DEFAULT_SENSOR_LOCAL_PORT), "local FT sensor port").asString();

    yarp::os::Property sensorOptions {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value(sensorRemote)},
        {"local", yarp::os::Value(sensorLocal)}
    };

    if (!sensorDevice.open(sensorOptions))
    {
        yCError(FTC) << "Failed to open sensor device";
        return false;
    }

    if (!sensorDevice.view(sensor))
    {
        yCError(FTC) << "Failed to view sensor interface";
        return false;
    }

    sensorIndex = -1;

    for (auto i = 0; i < sensor->getNrOfSixAxisForceTorqueSensors(); i++)
    {
        std::string temp;

        if (sensor->getSixAxisForceTorqueSensorName(i, temp) && temp == sensorName)
        {
            sensorIndex = i;
            break;
        }
    }

    if (sensorIndex == -1)
    {
        yCError(FTC) << "Failed to find sensor with name" << sensorName;
        return false;
    }

    return true;
}

bool FtCompensation::updateModule()
{
    return true;
}

bool FtCompensation::interruptModule()
{
    return true;
}

double FtCompensation::getPeriod()
{
    return 0.01; // [s]
}

bool FtCompensation::close()
{
    sensorDevice.close();
    return true;
}

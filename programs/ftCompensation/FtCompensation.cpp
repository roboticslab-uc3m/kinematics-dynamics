// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FtCompensation.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_SENSOR_LOCAL_PORT = "/ftCompensation";
constexpr auto DEFAULT_LIN_GAIN = "1.0";
constexpr auto DEFAULT_ROT_GAIN = "1.0";
constexpr auto DEFAULT_LIN_DEADBAND = "1.0";
constexpr auto DEFAULT_ROT_DEADBAND = "1.0";

bool FtCompensation::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(FTC) << "Config:" << rf.toString();

    linGain = rf.check("linGain", yarp::os::Value(DEFAULT_LIN_GAIN), "linear gain").asFloat64();
    rotGain = rf.check("rotGain", yarp::os::Value(DEFAULT_ROT_GAIN), "rotational gain").asFloat64();
    linDeadband = rf.check("linDeadband", yarp::os::Value(DEFAULT_LIN_DEADBAND), "linear deadband [N]").asFloat64();
    rotDeadband = rf.check("rotDeadband", yarp::os::Value(DEFAULT_ROT_DEADBAND), "rotational deadband [Nm]").asFloat64();

    auto vToolCoM = rf.check("toolCoM", yarp::os::Value::getNullValue(), "tool CoM regarding to TCP frame");
    auto vGravity = rf.check("gravity", yarp::os::Value::getNullValue(), "gravity vector regarding to inertial frame");

    if (!vToolCoM.isNull() && !vGravity.isNull())
    {
        if (!vToolCoM.isList() || vToolCoM.asList()->size() != 3)
        {
            yCError(FTC) << "toolCoM must be a list of 3 doubles";
            return false;
        }

        if (!vGravity.isList() || vGravity.asList()->size() != 3)
        {
            yCError(FTC) << "gravity must be a list of 3 doubles";
            return false;
        }

        toolCoM.x(vToolCoM.asList()->get(0).asFloat64());
        toolCoM.y(vToolCoM.asList()->get(1).asFloat64());
        toolCoM.z(vToolCoM.asList()->get(2).asFloat64());

        yCInfo(FTC) << "Tool CoM:" << vToolCoM.toString();

        gravity.x(vGravity.asList()->get(0).asFloat64());
        gravity.y(vGravity.asList()->get(1).asFloat64());
        gravity.z(vGravity.asList()->get(2).asFloat64());

        yCInfo(FTC) << "Gravity:" << vGravity.toString();
    }
    else
    {
        yCInfo(FTC) << "Using no tool";
    }

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

#include "StreamingDevice.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "SpnavSensorDevice.hpp"
#include "LeapMotionSensorDevice.hpp"
#include "WiimoteSensorDevice.hpp"

using namespace roboticslab;

StreamingDevice * StreamingDeviceFactory::makeDevice(const std::string & deviceName, yarp::os::Searchable & config)
{
    auto & deviceConfig = config.findGroup(deviceName.c_str());
    bool usingMovi = config.check("movi", "enable movi command");

    yDebug() << "Device configuration:" << deviceConfig.toString();

    if (deviceName == "SpaceNavigator")
    {
        double gain = config.check("gain", yarp::os::Value(0.0)).asFloat64();
        return new SpnavSensorDevice(deviceConfig, usingMovi, gain);
    }
    else if (deviceName == "LeapMotionSensor")
    {
        return new LeapMotionSensorDevice(deviceConfig, usingMovi);
    }
    else if (deviceName == "WiimoteSensor")
    {
        return new WiimoteSensorDevice(deviceConfig, usingMovi);
    }
    else
    {
        yError() << "Invalid device:" << deviceName;
        return new InvalidDevice();
    }
}

StreamingDevice::StreamingDevice(yarp::os::Searchable & config)
    : iCartesianControl(nullptr),
      actuatorState(VOCAB_CC_ACTUATOR_NONE)
{
    data.resize(6, 0.0);
    fixedAxes.resize(6, false);

    yarp::dev::PolyDriver::open(config);
    configureFixedAxes(config.find("fixedAxes"));
}

StreamingDevice::~StreamingDevice()
{
    yarp::dev::PolyDriver::close();
}

bool StreamingDevice::transformData(double scaling)
{
    for (int i = 0; i < 6; i++)
    {
        if (!fixedAxes[i])
        {
            data[i] /= scaling;
        }
        else
        {
            data[i] = 0.0;
        }
    }

    return true;
}

bool StreamingDevice::hasValidMovementData() const
{
    if (actuatorState != VOCAB_CC_ACTUATOR_NONE)
    {
        return false;
    }

    for (int i = 0; i < 6; i++)
    {
        if (data[i] != 0.0)
        {
            return true;
        }
    }

    return false;
}

void StreamingDevice::configureFixedAxes(const yarp::os::Value & v)
{
    if (!v.isList())
    {
        return;
    }

    yarp::os::Bottle * axesList = v.asList();

    for (int i = 0; i < axesList->size(); i++)
    {
        std::string str = axesList->get(i).asString();

        if (str == "x")
        {
            fixedAxes[0] = true;
        }
        else if (str == "y")
        {
            fixedAxes[1] = true;
        }
        else if (str == "z")
        {
            fixedAxes[2] = true;
        }
        else if (str == "rotx")
        {
            fixedAxes[3] = true;
        }
        else if (str == "roty")
        {
            fixedAxes[4] = true;
        }
        else if (str == "rotz")
        {
            fixedAxes[5] = true;
        }
        else
        {
            yWarning() << "Ignoring unrecognized fixed axis label:" << str;
        }
    }
}

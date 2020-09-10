#include "StreamingDevice.hpp"

#include <string>

#include <yarp/os/Property.h>

#include <ColorDebug.h>

#include "SpnavSensorDevice.hpp"
#include "LeapMotionSensorDevice.hpp"
#include "WiimoteSensorDevice.hpp"

using namespace roboticslab;

StreamingDevice * StreamingDeviceFactory::makeDevice(const std::string & deviceName, yarp::os::Searchable & config)
{
    yarp::os::Searchable & deviceConfig = config.findGroup(deviceName.c_str());
    bool usingMovi = config.check("movi", "enable movi command");

    CD_DEBUG("Device configuration: %s\n", deviceConfig.toString().c_str());

    if (deviceName == "SpaceNavigator")
    {
        double gain = config.check("gain", yarp::os::Value(0.0)).asFloat64();
        return new SpnavSensorDevice(deviceConfig, usingMovi, gain);
    }
    else if (deviceName == "LeapMotionSensor")
    {
        double period = config.check("period", yarp::os::Value(0.0)).asFloat64();
        return new LeapMotionSensorDevice(deviceConfig, usingMovi, period);
    }
    else if (deviceName == "WiimoteSensor")
    {
        return new WiimoteSensorDevice(deviceConfig, usingMovi);
    }
    else
    {
        CD_ERROR("Invalid device \"%s\".\n", deviceName.c_str());
        return new InvalidDevice();
    }
}

StreamingDevice::StreamingDevice(yarp::os::Searchable & config)
    : iCartesianControl(NULL),
      actuatorState(VOCAB_CC_ACTUATOR_NONE)
{
    data.resize(6, 0.0);
    fixedAxes.resize(6, false);

    PolyDriver::open(config);
    configureFixedAxes(config.find("fixedAxes"));
}

StreamingDevice::~StreamingDevice()
{
    PolyDriver::close();
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
            CD_WARNING("Unrecognized fixed axis label: %s. Ignoring...\n", str.c_str());
        }
    }
}

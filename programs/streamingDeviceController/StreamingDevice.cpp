#include "StreamingDevice.hpp"

#include <string>

#include <yarp/os/Property.h>

#include <ColorDebug.hpp>

#include "SpnavSensorDevice.hpp"

using namespace roboticslab;

StreamingDevice * StreamingDeviceFactory::makeDevice(const std::string & deviceName, yarp::os::Searchable & config)
{
    if (deviceName == "SpaceNavigator")
    {
        return new SpnavSensorDevice(config);
    }
    else
    {
        CD_ERROR("Invalid device \"%s\".\n", deviceName.c_str());
        return new InvalidDevice();
    }
}

StreamingDevice::StreamingDevice(yarp::os::Searchable & config)
    : iCartesianControl(NULL)
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
    for (int i = 0; i < data.size(); i++)
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
    for (int i = 0; i < data.size(); i++)
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

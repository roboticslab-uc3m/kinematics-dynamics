#include "StreamingDevice.hpp"

#include <string>

#include <yarp/os/Property.h>

#include <ColorDebug.hpp>

#include "SpnavSensorDevice.hpp"

roboticslab::StreamingDevice * roboticslab::StreamingDeviceFactory::makeDevice(yarp::os::Searchable & config)
{
    const std::string prop = "streamingDevice";
    std::string deviceName = config.check(prop.c_str(), yarp::os::Value::getNullValue(), "device name").asString();

    yarp::os::Property options;
    options.fromString(config.toString());
    options.unput(prop.c_str());

    if (deviceName == "SpaceNavigator")
    {
        options.put("device", "analogsensorclient");
        return new SpnavSensorDevice(config);
    }
    else if (deviceName.empty())
    {
        CD_ERROR("Missing or empty parameter \"%s\".\n", prop.c_str());
        return new InvalidDevice();
    }
    else
    {
        CD_ERROR("Invalid device \"%s\".\n", deviceName.c_str());
        return new InvalidDevice();
    }
}

roboticslab::StreamingDevice::StreamingDevice(yarp::os::Searchable & config)
    : iCartesianControl(NULL)
{
    data.resize(6, 0.0);
    fixedAxes.resize(6, false);

    PolyDriver::open(config);
    configureFixedAxes(config.find("fixedAxes"));
}

roboticslab::StreamingDevice::~StreamingDevice()
{
    PolyDriver::close();
}

bool roboticslab::StreamingDevice::transformData(double scaling)
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

void roboticslab::StreamingDevice::configureFixedAxes(const yarp::os::Value & v)
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

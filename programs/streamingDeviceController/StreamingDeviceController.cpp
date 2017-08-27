#include "StreamingDeviceController.hpp"

#include <string>
#include <cmath>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <yarp/sig/Vector.h>

#include <ColorDebug.hpp>

namespace roboticslab
{

bool StreamingDeviceController::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("streamingDeviceController config: %s.\n", rf.toString().c_str());

    std::string localDevice = rf.check("localDevice", yarp::os::Value(DEFAULT_DEVICE_PORT_LOCAL), "local device port").asString();
    std::string remoteDevice = rf.check("remoteDevice", yarp::os::Value(DEFAULT_DEVICE_PORT_REMOTE), "remote device port").asString();

    std::string localActuator = rf.check("localActuator", yarp::os::Value(DEFAULT_ACTUATOR_LOCAL), "local actuator port").asString();
    std::string remoteActuator = rf.check("remoteActuator", yarp::os::Value(DEFAULT_ACTUATOR_REMOTE), "remote actuator port").asString();

    std::string localCartesian = rf.check("localCartesian", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL), "local cartesian port").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE), "remote cartesian port").asString();

    std::string sensorsPort = rf.check("sensorsPort", yarp::os::Value(DEFAULT_PROXIMITY_SENSORS), "remote sensors port").asString();

    scaling = rf.check("scaling", yarp::os::Value(DEFAULT_SCALING), "scaling factor").asDouble();

    yarp::os::Value axesValue = rf.check("fixedAxes", yarp::os::Value(DEFAULT_FIXED_AXES), "axes with restricted movement");

    fixedAxes.resize(6, false);

    if (axesValue.isList())
    {
        yarp::os::Bottle * axesList = axesValue.asList();

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

    if (rf.check("help"))
    {
        printf("StreamingDeviceController options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        return false;
    }

    yarp::os::Property streamingClientOptions;
    streamingClientOptions.put("device", "analogsensorclient");
    streamingClientOptions.put("local", localDevice);
    streamingClientOptions.put("remote", remoteDevice);

    streamingClientDevice.open(streamingClientOptions);

    if (!streamingClientDevice.isValid())
    {
        CD_ERROR("spnav client device not valid.\n");
        return false;
    }

    if (!streamingClientDevice.view(iAnalogSensor))
    {
        CD_ERROR("Could not view iAnalogSensor.\n");
        return false;
    }

    yarp::os::Property actuatorClientOptions;
    actuatorClientOptions.put("device", "analogsensorclient");
    actuatorClientOptions.put("local", localActuator);
    actuatorClientOptions.put("remote", remoteActuator);

    actuatorClientDevice.open(actuatorClientOptions);

    if (!actuatorClientDevice.isValid())
    {
        CD_ERROR("actuator client device not valid.\n");
        return false;
    }

    if (!actuatorClientDevice.view(iAnalogSensorAct))
    {
        CD_ERROR("Could not view iAnalogSensorAct.\n");
        return false;
    }

    yarp::os::Property cartesianControlClientOptions;
    cartesianControlClientOptions.put("device", "CartesianControlClient");
    cartesianControlClientOptions.put("cartesianLocal", localCartesian);
    cartesianControlClientOptions.put("cartesianRemote", remoteCartesian);

    cartesianControlClientDevice.open(cartesianControlClientOptions);

    if (!cartesianControlClientDevice.isValid())
    {
        CD_ERROR("cartesian control client device not valid.\n");
        return false;
    }

    if (!cartesianControlClientDevice.view(iCartesianControl))
    {
        CD_ERROR("Could not view iCartesianControl.\n");
        return false;
    }

    if (rf.check("useSensors"))
    {
        yarp::os::Property sensorsClientOptions;
        sensorsClientOptions.fromString(rf.toString());
        sensorsClientOptions.put("device", "ProximitySensorsClient");

        sensorsClientDevice.open(sensorsClientOptions);

        if (!sensorsClientDevice.isValid())
        {
            CD_ERROR("sensors device not valid.\n");
            return false;
        }

        if (!sensorsClientDevice.view(iProximitySensors))
        {
            CD_ERROR("Could not view iSensors.\n");
            return false;
        }
    }

    isStopped = true;
    actuatorState = 2;

    return true;
}

bool StreamingDeviceController::updateModule()
{
    yarp::sig::Vector data;

    iAnalogSensorAct->read(data);

    if (data.size() == 2)
    {
        int button1 = data[0];
        int button2 = data[1];

        if (button1 == 1)
        {
            actuatorState = 0;
            iCartesianControl->act(0);
        }
        else if (button2 == 1)
        {
            actuatorState = 1;
            iCartesianControl->act(1);
        }
        else
        {
            if (actuatorState != 2)
            {
                iCartesianControl->act(2);
            }

            actuatorState = 2;
        }
    }

    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 6)
    {
        CD_ERROR("Invalid data size: %d.\n", data.size());
        return false;
    }

    std::vector<double> xdot(6, 0.0);
    bool isZero = true;

    double local_scaling = scaling;

    if (sensorsClientDevice.isValid() && iProximitySensors->getAlertLevel() == IProximitySensors::LOW)
    {
        local_scaling *= 2; //Half velocity
        CD_WARNING("Obstacle detected\n");
    }

    for (int i = 0; i < data.size(); i++)
    {
        if (!fixedAxes[i] && data[i] != 0.0)
        {
            isZero = false;
            xdot[i] = data[i] / scaling;
        }
    }

    if (isZero || (sensorsClientDevice.isValid() && iProximitySensors->getAlertLevel() == IProximitySensors::HIGH))
    {
        if (!isStopped)
        {
            isStopped = iCartesianControl->stopControl();
        }

        return true;
    }
    else
    {
        isStopped = false;
    }

    if (!iCartesianControl->vmos(xdot))
    {
        CD_WARNING("vmos failed.\n");
    }

    return true;
}

bool StreamingDeviceController::interruptModule()
{
    bool ok = true;

    ok &= iCartesianControl->stopControl();
    ok &= cartesianControlClientDevice.close();
    ok &= streamingClientDevice.close();
    ok &= actuatorClientDevice.close();
    ok &= sensorsClientDevice.close();

    return ok;
}

double StreamingDeviceController::getPeriod()
{
    return 0.02;  // [s]
}

}  // namespace roboticslab

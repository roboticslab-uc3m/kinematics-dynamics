#include "StreamingDeviceController.hpp"

#include <string>
#include <cmath>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <yarp/sig/Vector.h>

#include <ColorDebug.hpp>

using namespace roboticslab;

bool StreamingDeviceController::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("streamingDeviceController config: %s.\n", rf.toString().c_str());

    std::string deviceName = rf.check("streamingDevice", yarp::os::Value(DEFAULT_DEVICE_NAME),
            "device name").asString();
    std::string localCartesian = rf.check("localCartesian", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),
            "local cartesian port").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
            "remote cartesian port").asString();
    std::string sensorsPort = rf.check("sensorsPort", yarp::os::Value(DEFAULT_PROXIMITY_SENSORS),
            "remote sensors port").asString();

    period = rf.check("controllerPeriod", yarp::os::Value(DEFAULT_PERIOD), "data acquisition period").asDouble();
    scaling = rf.check("scaling", yarp::os::Value(DEFAULT_SCALING), "scaling factor").asDouble();

    streamingDevice = StreamingDeviceFactory::makeDevice(deviceName, rf);

    if (!streamingDevice->isValid())
    {
        CD_ERROR("Streaming device not valid.\n");
        return false;
    }

    if (!streamingDevice->acquireInterfaces())
    {
        CD_ERROR("Unable to acquire plugin interfaces from streaming device.\n");
        return false;
    }

    yarp::os::Property cartesianControlClientOptions;
    cartesianControlClientOptions.put("device", "CartesianControlClient");
    cartesianControlClientOptions.put("cartesianLocal", localCartesian);
    cartesianControlClientOptions.put("cartesianRemote", remoteCartesian);

    cartesianControlClientDevice.open(cartesianControlClientOptions);

    if (!cartesianControlClientDevice.isValid())
    {
        CD_ERROR("Cartesian control client device not valid.\n");
        cartesianControlClientDevice.close(); // release managed resources
        return false;
    }

    if (!cartesianControlClientDevice.view(iCartesianControl))
    {
        CD_ERROR("Could not view iCartesianControl.\n");
        cartesianControlClientDevice.close(); // close ports
        return false;
    }

    streamingDevice->setCartesianControllerHandle(iCartesianControl);

    if (!streamingDevice->initialize())
    {
        CD_ERROR("Device initialization failed.\n");
        cartesianControlClientDevice.close(); // close ports
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

    return true;
}

bool StreamingDeviceController::updateModule()
{
    if (!streamingDevice->acquireData())
    {
        CD_ERROR("Failed to acquire data from streaming device.\n");
        return true;
    }

    double localScaling = scaling;

    if (sensorsClientDevice.isValid() && iProximitySensors->getAlertLevel() == IProximitySensors::LOW)
    {
        localScaling *= 2; //Half velocity
        CD_WARNING("Obstacle detected\n");
    }

    if (!streamingDevice->transformData(localScaling))
    {
        CD_ERROR("Failed to transform acquired data from streaming device.\n");
        return true;
    }

    if (!streamingDevice->hasValidMovementData() || (sensorsClientDevice.isValid() && iProximitySensors->getAlertLevel() == IProximitySensors::HIGH))
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

    streamingDevice->sendMovementCommand();

    return true;
}

bool StreamingDeviceController::interruptModule()
{
    iCartesianControl->stopControl();

    delete streamingDevice;
    streamingDevice = NULL;

    bool ok = true;

    ok &= cartesianControlClientDevice.close();

    if (sensorsClientDevice.isValid())
    {
        ok &= sensorsClientDevice.close();
    }

    return ok;
}

double StreamingDeviceController::getPeriod()
{
    return period;
}

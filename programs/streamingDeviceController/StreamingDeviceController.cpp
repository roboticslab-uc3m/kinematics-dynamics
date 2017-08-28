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

    period = rf.check("controllerPeriod", yarp::os::Value(DEFAULT_PERIOD), "data acquisition period").asDouble();
    scaling = rf.check("scaling", yarp::os::Value(DEFAULT_SCALING), "scaling factor").asDouble();

    streamingDevice = StreamingDeviceFactory::makeDevice(deviceName, rf.findGroup(deviceName.c_str()));

    if (!streamingDevice->isValid())
    {
        CD_ERROR("Streaming device not valid.\n");
        return false;
    }

    if (!streamingDevice->acquireInterfaces())
    {
        CD_ERROR("Unable to acquire plugin interfaces for streaming device.\n");
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

    if (!streamingDevice->transformData(scaling))
    {
        CD_ERROR("Failed to transform acquired data from streaming device.\n");
        return true;
    }

    if (!streamingDevice->hasValidMovementData())
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

    if (!streamingDevice->sendMovementCommand())
    {
        CD_WARNING("Failed to send movement command to cartesian controller.\n");
    }

    return true;
}

bool StreamingDeviceController::interruptModule()
{
    iCartesianControl->stopControl();

    delete streamingDevice;
    streamingDevice = NULL;

    return cartesianControlClientDevice.close();
}

double StreamingDeviceController::getPeriod()
{
    return period;
}

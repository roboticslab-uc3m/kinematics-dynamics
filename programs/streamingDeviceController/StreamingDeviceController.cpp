#include "StreamingDeviceController.hpp"

#include <cmath>
#include <string>
#include <typeinfo>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <yarp/sig/Vector.h>

#include <ColorDebug.h>

#include "SpnavSensorDevice.hpp" // for typeid() check

using namespace roboticslab;

#ifdef SDC_WITH_SENSORS
const double roboticslab::StreamingDeviceController::SCALING_FACTOR_ON_ALERT = 2.0;
#endif  // SDC_WITH_SENSORS

bool StreamingDeviceController::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("streamingDeviceController config: %s.\n", rf.toString().c_str());

    std::string deviceName = rf.check("streamingDevice", yarp::os::Value(DEFAULT_DEVICE_NAME),
            "device name").asString();
    std::string localCartesian = rf.check("localCartesian", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),
            "local cartesian port").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
            "remote cartesian port").asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD), "data acquisition period").asFloat64();
    scaling = rf.check("scaling", yarp::os::Value(DEFAULT_SCALING), "scaling factor").asFloat64();

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
        return false;
    }

    if (!cartesianControlClientDevice.view(iCartesianControl))
    {
        CD_ERROR("Could not view iCartesianControl.\n");
        return false;
    }

    std::map<int, double> params;

    if (!iCartesianControl->getParameters(params))
    {
        CD_ERROR("Unable to retrieve configuration parameters.\n");
        return false;
    }

    bool usingStreamingPreset = params.find(VOCAB_CC_CONFIG_STREAMING_CMD) != params.end();

    streamingDevice->setCartesianControllerHandle(iCartesianControl);

    if (!streamingDevice->initialize(usingStreamingPreset))
    {
        CD_ERROR("Device initialization failed.\n");
        return false;
    }

#ifdef SDC_WITH_SENSORS
    if (rf.check("useSensors", "enable proximity sensors"))
    {
        std::string sensorsPort = rf.check("sensorsPort", yarp::os::Value(DEFAULT_PROXIMITY_SENSORS),
                "remote sensors port").asString();

        yarp::os::Property sensorsClientOptions;
        sensorsClientOptions.fromString(rf.toString());
        sensorsClientOptions.put("device", "ProximitySensorsClient");
        sensorsClientOptions.put("remote", sensorsPort);

        sensorsClientDevice.open(sensorsClientOptions);

        if (!sensorsClientDevice.isValid())
        {
            CD_ERROR("Sensors device not valid.\n");
            return false;
        }

        if (!sensorsClientDevice.view(iProximitySensors))
        {
            CD_ERROR("Could not view iSensors.\n");
            return false;
        }

        disableSensorsLowLevel = rf.check("disableSensorsLowLevel");
    }
#endif  // SDC_WITH_SENSORS

    if (rf.check("remoteCentroid", "remote centroid port"))
    {
        const std::type_info & spnavType = typeid(SpnavSensorDevice);

        if (typeid(*streamingDevice) != spnavType)
        {
            CD_ERROR("Centroid transform extension only available with %s.\n", spnavType.name());
            return false;
        }

        std::string localCentroid = rf.check("localCentroid", yarp::os::Value(DEFAULT_CENTROID_LOCAL),
                    "local centroid port").asString();
        std::string remoteCentroid = rf.check("remoteCentroid", yarp::os::Value::getNullValue()).asString();

        yarp::os::Value vCentroidRPY = rf.check("centroidRPY", yarp::os::Value::getNullValue());

        double permanenceTime = rf.check("centroidPermTime", yarp::os::Value(0.0)).asFloat64();

        if (!vCentroidRPY.isNull() && vCentroidRPY.isList() && !centroidTransform.setTcpToCameraRotation(vCentroidRPY.asList()))
        {
            CD_ERROR("Illegal argument: malformed bottle --centroidRPY: %s.\n", vCentroidRPY.toString().c_str());
            return false;
        }

        if (permanenceTime < 0.0)
        {
            CD_ERROR("Illegal argument: --centroidPermTime cannot be less than zero: %f.\n", permanenceTime);
            return false;
        }
        else
        {
            centroidTransform.setPermanenceTime(permanenceTime);
        }

        if (!centroidPort.open(localCentroid + "/state:i"))
        {
            CD_ERROR("Unable to open local centroid port.\n");
            return false;
        }

        if (!yarp::os::Network::connect(remoteCentroid, centroidPort.getName(), "udp"))
        {
            CD_ERROR("Unable to connect to %s.\n", remoteCentroid.c_str());
            return false;
        }

        centroidTransform.registerStreamingDevice(streamingDevice);
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

#ifdef SDC_WITH_SENSORS
    IProximitySensors::alert_level alertLevel = IProximitySensors::ZERO;

    if (sensorsClientDevice.isValid())
    {
        alertLevel = iProximitySensors->getAlertLevel();

        if (!disableSensorsLowLevel && alertLevel == IProximitySensors::LOW)
        {
            localScaling *= SCALING_FACTOR_ON_ALERT;
            CD_WARNING("Obstacle (low level) - decrease speed, factor %f.\n", SCALING_FACTOR_ON_ALERT);
        }
        else if (alertLevel == IProximitySensors::HIGH)
        {
            CD_WARNING("Obstacle (high level) - command stop.\n");

            if (!isStopped)
            {
                streamingDevice->stopMotion();
                isStopped = true;
            }

            return true;
        }
    }
#endif  // SDC_WITH_SENSORS

    int actuatorState = streamingDevice->getActuatorState();

    if (actuatorState != VOCAB_CC_ACTUATOR_NONE)
    {
        iCartesianControl->act(actuatorState);
    }

    if (!streamingDevice->transformData(localScaling))
    {
        CD_ERROR("Failed to transform acquired data from streaming device.\n");
        return true;
    }

    if (!centroidPort.isClosed())
    {
        yarp::os::Bottle * centroidBottle = centroidPort.read(false);

        if (centroidTransform.acceptBottle(centroidBottle) && centroidTransform.processStoredBottle())
        {
            CD_WARNING("Centroid transform handler takes control.\n");
        }
    }

    if (streamingDevice->hasValidMovementData())
    {
        streamingDevice->sendMovementCommand();
        isStopped = false;
    }
    else
    {
        if (!isStopped)
        {
            streamingDevice->stopMotion();
        }

        isStopped = true;
    }

    return true;
}

bool StreamingDeviceController::interruptModule()
{
    return iCartesianControl->stopControl();
}

bool StreamingDeviceController::close()
{
    delete streamingDevice;
    streamingDevice = NULL;

    bool ok = cartesianControlClientDevice.close();

#ifdef SDC_WITH_SENSORS
    ok &= sensorsClientDevice.close();
#endif  // SDC_WITH_SENSORS

    centroidPort.close();

    return ok;
}

double StreamingDeviceController::getPeriod()
{
    return period;
}

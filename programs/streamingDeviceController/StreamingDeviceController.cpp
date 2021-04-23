#include "StreamingDeviceController.hpp"

#include <cmath>
#include <string>
#include <typeinfo>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <yarp/sig/Vector.h>

#include "SpnavSensorDevice.hpp" // for typeid() check

using namespace roboticslab;

#ifdef SDC_WITH_SENSORS
const double roboticslab::StreamingDeviceController::SCALING_FACTOR_ON_ALERT = 2.0;
#endif  // SDC_WITH_SENSORS

bool StreamingDeviceController::configure(yarp::os::ResourceFinder &rf)
{
    yDebug() << "streamingDeviceController config:" << rf.toString();

    std::string localPrefix = rf.check("prefix", yarp::os::Value(DEFAULT_LOCAL_PREFIX),
            "local port name prefix").asString();
    std::string deviceName = rf.check("streamingDevice", yarp::os::Value(DEFAULT_DEVICE_NAME),
            "device name").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(""),
            "remote cartesian port").asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD), "data acquisition period").asFloat64();
    scaling = rf.check("scaling", yarp::os::Value(DEFAULT_SCALING), "scaling factor").asFloat64();

    streamingDevice = StreamingDeviceFactory::makeDevice(deviceName, rf);

    if (!streamingDevice->isValid())
    {
        yError() << "Streaming device not valid";
        return false;
    }

    if (!streamingDevice->acquireInterfaces())
    {
        yError() << "Unable to acquire plugin interfaces from streaming device";
        return false;
    }

    yarp::os::Property cartesianControlClientOptions;
    cartesianControlClientOptions.put("device", "CartesianControlClient");
    cartesianControlClientOptions.put("cartesianLocal", localPrefix + "/cartesian");
    cartesianControlClientOptions.put("cartesianRemote", remoteCartesian);

    cartesianControlClientDevice.open(cartesianControlClientOptions);

    if (!cartesianControlClientDevice.isValid())
    {
        yError() << "Cartesian control client device not valid";
        return false;
    }

    if (!cartesianControlClientDevice.view(iCartesianControl))
    {
        yError() << "Could not view iCartesianControl";
        return false;
    }

    std::map<int, double> params;

    if (!iCartesianControl->getParameters(params))
    {
        yError() << "Unable to retrieve configuration parameters";
        return false;
    }

    bool usingStreamingPreset = params.find(VOCAB_CC_CONFIG_STREAMING_CMD) != params.end();

    streamingDevice->setCartesianControllerHandle(iCartesianControl);

    if (!streamingDevice->initialize(usingStreamingPreset))
    {
        yError() << "Device initialization failed";
        return false;
    }

#ifdef SDC_WITH_SENSORS
    if (rf.check("useSensors", "enable proximity sensors"))
    {
        std::string sensorsPort = rf.check("sensorsPort", yarp::os::Value("/sensor_reader"),
                "remote sensors port").asString();

        yarp::os::Property sensorsClientOptions;
        sensorsClientOptions.fromString(rf.toString());
        sensorsClientOptions.put("device", "ProximitySensorsClient");
        sensorsClientOptions.put("remote", sensorsPort);

        sensorsClientDevice.open(sensorsClientOptions);

        if (!sensorsClientDevice.isValid())
        {
            yError() << "Sensors device not valid";
            return false;
        }

        if (!sensorsClientDevice.view(iProximitySensors))
        {
            yError() << "Could not view iSensors";
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
            yError() << "Centroid transform extension only available with" << spnavType.name();
            return false;
        }

        std::string remoteCentroid = rf.check("remoteCentroid", yarp::os::Value::getNullValue()).asString();
        yarp::os::Value vCentroidRPY = rf.check("centroidRPY", yarp::os::Value::getNullValue());
        double permanenceTime = rf.check("centroidPermTime", yarp::os::Value(0.0)).asFloat64();

        if (!vCentroidRPY.isNull() && vCentroidRPY.isList() && !centroidTransform.setTcpToCameraRotation(vCentroidRPY.asList()))
        {
            yError() << "Illegal argument: malformed bottle --centroidRPY:" << vCentroidRPY.toString();
            return false;
        }

        if (permanenceTime < 0.0)
        {
            yError() << "Illegal argument: --centroidPermTime cannot be less than zero:" << permanenceTime;
            return false;
        }
        else
        {
            centroidTransform.setPermanenceTime(permanenceTime);
        }

        if (!centroidPort.open(localPrefix + "/centroid/state:i"))
        {
            yError() << "Unable to open local centroid port";
            return false;
        }

        if (!yarp::os::Network::connect(remoteCentroid, centroidPort.getName(), "udp"))
        {
            yError() << "Unable to connect to" << remoteCentroid;
            return false;
        }

        centroidTransform.registerStreamingDevice(streamingDevice);
    }

    isStopped = true;

    if (rf.check("syncPort", "remote synchronization port"))
    {
        auto remoteSync = rf.find("syncPort").asString();

        if (!syncPort.open(localPrefix + "/sync:i"))
        {
            yError() << "Unable to open local sync port";
            return false;
        }

        if (!yarp::os::Network::connect(remoteSync, syncPort.getName(), "fast_tcp"))
        {
            yError() << "Unable to connect to remote sync port" << remoteSync;
            return false;
        }

        syncPort.useCallback(*this);
    }

    return true;
}

bool StreamingDeviceController::updateModule()
{
    if (syncPort.isClosed())
    {
        auto now = yarp::os::Time::now();
        return update(now);
    }

    return true;
}

void StreamingDeviceController::onRead(yarp::os::Bottle & bot)
{
    if (bot.size() == 2)
    {
        double now = bot.get(0).asInt32() + bot.get(1).asInt32() * 1e-9;
        update(now);
    }
    else
    {
        yWarning() << "Illegal bottle size";
    }
}

bool StreamingDeviceController::update(double timestamp)
{
    if (!streamingDevice->acquireData())
    {
        yError() << "Failed to acquire data from streaming device";
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
            yWarning() << "Obstacle (low level) - decrease speed, factor" << SCALING_FACTOR_ON_ALERT;
        }
        else if (alertLevel == IProximitySensors::HIGH)
        {
            yWarning() << "Obstacle (high level) - command stop";

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
        yError() << "Failed to transform acquired data from streaming device";
        return true;
    }

    if (!centroidPort.isClosed())
    {
        yarp::os::Bottle * centroidBottle = centroidPort.read(false);

        if (centroidTransform.acceptBottle(centroidBottle) && centroidTransform.processStoredBottle())
        {
            yWarning() << "Centroid transform handler takes control";
        }
    }

    if (streamingDevice->hasValidMovementData())
    {
        streamingDevice->sendMovementCommand(timestamp);
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
    if (!syncPort.isClosed())
    {
        syncPort.interrupt();
        syncPort.disableCallback();
    }

    return iCartesianControl->stopControl();
}

bool StreamingDeviceController::close()
{
    if (!syncPort.isClosed())
    {
        syncPort.close();
    }

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

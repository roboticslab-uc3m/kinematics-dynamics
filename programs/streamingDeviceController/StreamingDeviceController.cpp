#include "StreamingDeviceController.hpp"

#include <cmath>

#include <sstream>
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

constexpr auto DEFAULT_DEVICE_NAME = "SpaceNavigator";
constexpr auto DEFAULT_LOCAL_PREFIX = "/streamingDeviceController";
constexpr auto DEFAULT_PERIOD = 0.1;
constexpr auto DEFAULT_SCALING = 10.0;

constexpr auto DEFAULT_PORTMONITOR_TYPE = "lua";
constexpr auto DEFAULT_PORTMONITOR_CONTEXT = "sensors";
constexpr auto DEFAULT_PORTMONITOR_FILE = "amor_sensors_modifier";

constexpr auto DEFAULT_THRESHOLD_ALERT_HIGH = 800;
constexpr auto DEFAULT_THRESHOLD_ALERT_LOW = 100;
constexpr auto SCALING_FACTOR_ON_ALERT = 2.0;

bool StreamingDeviceController::configure(yarp::os::ResourceFinder &rf)
{
    yDebug() << "streamingDeviceController config:" << rf.toString();

    auto localPrefix = rf.check("prefix", yarp::os::Value(DEFAULT_LOCAL_PREFIX), "local port name prefix").asString();
    auto deviceName = rf.check("streamingDevice", yarp::os::Value(DEFAULT_DEVICE_NAME), "device name").asString();
    auto remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(""), "remote cartesian port").asString();

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

    yarp::os::Property cartesianControlClientOptions {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"cartesianLocal", yarp::os::Value(localPrefix + "/cartesian")},
        {"cartesianRemote", yarp::os::Value(remoteCartesian)}
    };

    if (!cartesianControlClientDevice.open(cartesianControlClientOptions))
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

    if (rf.check("sensorsPort", "remote sensors port"))
    {
        std::string carrier;

        if (rf.check("usePortMonitor", "enable port monitoring and additional options"))
        {
            auto pmType = rf.check("portMonitorType", yarp::os::Value(DEFAULT_PORTMONITOR_TYPE), "port monitor type").asString();
            auto pmContext = rf.check("portMonitorContext", yarp::os::Value(DEFAULT_PORTMONITOR_CONTEXT), "port monitor context").asString();
            auto pmFile = rf.check("portMonitorFile", yarp::os::Value(DEFAULT_PORTMONITOR_FILE), "port monitor file").asString();

            std::ostringstream oss;
            oss << "tcp+recv.portmonitor+type." << pmType << "+context." << pmContext << "+file." << pmFile;

            carrier = oss.str();
            yInfo() << "Using carrier:" << carrier;
        }

        thresholdAlertHigh = rf.check("thresholdAlertHigh", yarp::os::Value(DEFAULT_THRESHOLD_ALERT_HIGH),
                "sensor threshold (proximity alert, high level)").asFloat64();
        thresholdAlertLow = rf.check("thresholdAlertLow", yarp::os::Value(DEFAULT_THRESHOLD_ALERT_LOW),
                "sensor threshold (proximity alert, low level)").asFloat64();

        disableSensorsLowLevel = rf.check("disableSensorsLowLevel");

        auto sensorsPort = rf.find("sensorsPort").asString();

        if (!proximityPort.open(localPrefix + "/proximity:i"))
        {
            yError() << "Unable to open local proximity port";
            return false;
        }

        if (!yarp::os::Network::connect(sensorsPort, proximityPort.getName(), carrier))
        {
            yError() << "Unable to connect to" << sensorsPort;
            return false;
        }
    }

    if (rf.check("remoteCentroid", "remote centroid port"))
    {
        const auto & spnavType = typeid(SpnavSensorDevice);

        if (typeid(*streamingDevice) != spnavType)
        {
            yError() << "Centroid transform extension only available with" << spnavType.name();
            return false;
        }

        auto remoteCentroid = rf.check("remoteCentroid", yarp::os::Value::getNullValue()).asString();
        auto vCentroidRPY = rf.check("centroidRPY", yarp::os::Value::getNullValue());
        auto permanenceTime = rf.check("centroidPermTime", yarp::os::Value(0.0)).asFloat64();

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

    if (!proximityPort.isClosed())
    {
        const auto * bottle = proximityPort.read(false);

        if (bottle && bottle->size() != 0)
        {
            double alertLevel = 0.0;

            for (int i = 0; i < bottle->size(); i++)
            {
                if (bottle->get(i).asFloat64() > alertLevel)
                {
                    alertLevel = bottle->get(i).asFloat64();
                }
            }

            if (alertLevel > thresholdAlertHigh)
            {
                yWarning() << "Obstacle (high level) - command stop";

                if (!isStopped)
                {
                    streamingDevice->stopMotion();
                    isStopped = true;
                }

                return true;
            }
            else if (!disableSensorsLowLevel && alertLevel > thresholdAlertLow)
            {
                localScaling *= SCALING_FACTOR_ON_ALERT;
                yWarning() << "Obstacle (low level) - decrease speed, factor" << SCALING_FACTOR_ON_ALERT;
            }
        }
    }

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
        auto * centroidBottle = centroidPort.read(false);

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
    if (!proximityPort.isClosed())
    {
        proximityPort.interrupt();
        proximityPort.close();
    }

    if (!centroidPort.isClosed())
    {
        centroidPort.interrupt();
        centroidPort.close();
    }

    if (!syncPort.isClosed())
    {
        syncPort.interrupt();
        syncPort.close();
    }

    delete streamingDevice;
    streamingDevice = nullptr;

    return cartesianControlClientDevice.close();
}

double StreamingDeviceController::getPeriod()
{
    return period;
}

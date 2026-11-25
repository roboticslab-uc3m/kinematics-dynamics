#include "HapticFeedback.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_LOCAL_PORT_PREFIX = "/hapticFeedback";
constexpr auto DEFAULT_PERIOD = 0.01;
constexpr auto DEFAULT_CARTESIAN_DEVICE = "CartesianControlClient";
constexpr auto DEFAULT_SCALING = 0.001;

bool HapticFeedback::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(HF) << "Config:" << rf.toString();

    auto local = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PORT_PREFIX), "local port prefix").asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD), "module period in seconds").asFloat64();

    if (period <= 0.0)
    {
        yCError(HF) << "Invalid period specified:" << period;
        return false;
    }

    scaling = rf.check("scaling", yarp::os::Value(DEFAULT_SCALING), "scaling factor for sensor measurements").asFloat64();

    if (scaling <= 0.0)
    {
        yCError(HF) << "Invalid scaling factor specified:" << scaling;
        return false;
    }

    const auto & v = rf.find("deviceFrameRPY");

    if (!v.isNull())
    {
        const auto * deviceFrameRPY = v.asList();

        if (!deviceFrameRPY->isNull() && deviceFrameRPY->size() == 3)
        {
            auto roll = deviceFrameRPY->get(0).asFloat64() * KDL::deg2rad;
            auto pitch = deviceFrameRPY->get(1).asFloat64() * KDL::deg2rad;
            auto yaw = deviceFrameRPY->get(2).asFloat64() * KDL::deg2rad;

            yCInfo(HF) << "deviceFrameRPY [rad]:" << roll << pitch << yaw;

            KDL::Frame H_sensor_device = KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw));
            H_device_sensor = H_sensor_device.Inverse();
        }
        else
        {
            yCError(HF) << "Invalid deviceFrameRPY specified:" << v.toString();
            return false;
        }
    }

    {
        auto remoteSensor = rf.check("remoteSensor", yarp::os::Value(""), "remote sensor port prefix").asString();

        if (remoteSensor.empty())
        {
            yCError(HF) << "Remote sensor port prefix not specified";
            return false;
        }

        yarp::os::Property options {
            {"device", yarp::os::Value("multipleanalogsensorsclient")},
            {"local", yarp::os::Value(local + "/mas")},
            {"remote", yarp::os::Value(remoteSensor)}
        };

        if (!sensorClientDevice.open(options))
        {
            yCError(HF) << "Failed to open sensor client device";
            return false;
        }

        if (!sensorClientDevice.view(iSensor))
        {
            yCError(HF) << "Failed to view ISixAxisForceTorqueSensors interface";
            return false;
        }

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
        std::size_t numSensors;

        if (!iSensor->getNrOfSixAxisForceTorqueSensors(numSensors))
        {
            yCError(HF) << "Failed to get number of sensors";
            return false;
        }
#else
        auto numSensors = iSensor->getNrOfSixAxisForceTorqueSensors();
#endif

        if (numSensors != 1)
        {
            yCError(HF) << "Expected exactly one sensor, but found" << numSensors;
            return false;
        }
    }

    {
        auto remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(""), "remote cartesian port prefix").asString();

        if (remoteCartesian.empty())
        {
            yCError(HF) << "Remote cartesian port prefix not specified";
            return false;
        }

        auto cartesianDevice = rf.check("cartesianDevice", yarp::os::Value(DEFAULT_CARTESIAN_DEVICE), "cartesian device name").asString();

        yarp::os::Property options {
            {"device", yarp::os::Value(cartesianDevice)},
            {"local", yarp::os::Value(local + "/cartesian")},
            {"remote", yarp::os::Value(remoteCartesian)}
        };

        if (!cartesianClientDevice.open(options))
        {
            yCError(HF) << "Failed to open cartesian client device";
            return false;
        }

        if (!cartesianClientDevice.view(iCartesianControl))
        {
            yCError(HF) << "Failed to view ICartesianControl interface";
            return false;
        }

        ICartesianControl::config_map_t params;

        if (!iCartesianControl->getParameters(params))
        {
            yCError(HF) << "Unable to retrieve configuration parameters";
            return false;
        }

        if (params.find(ICartesianControl::Config::STREAMING_CMD) != params.end())
        {
            if (auto value = static_cast<yarp::conf::vocab32_t>(ICartesianControl::Streaming::POSE);
                std::get<yarp::conf::vocab32_t>(params.at(ICartesianControl::Config::STREAMING_CMD)) != value &&
                !iCartesianControl->setParameter(ICartesianControl::Config::STREAMING_CMD, value))
            {
                yCError(HF) << "Unable to preset streaming command";
                return false;
            }
        }

        if (params.find(ICartesianControl::Config::FRAME) != params.end())
        {
            if (auto value = static_cast<yarp::conf::vocab32_t>(ICartesianSolver::Frame::BASE);
                std::get<yarp::conf::vocab32_t>(params.at(ICartesianControl::Config::FRAME)) != value &&
                !iCartesianControl->setParameter(ICartesianControl::Config::FRAME, value))
            {
                yCError(HF) << "Unable to set inertial reference frame";
                return false;
            }
        }
    }

    return true;
}

bool HapticFeedback::updateModule()
{
    if (!iSensor->getSixAxisForceTorqueSensorMeasure(0, sensorMeasure, sensorTimestamp))
    {
        yCError(HF) << "Failed to get sensor measure";
        return false;
    }

    std::vector<double> reaction(sensorMeasure.size());

    for (auto i = 0; i < sensorMeasure.size(); i++)
    {
        reaction[i] = sensorMeasure[i] * scaling;
    }

    yCDebug(HF) << "Reaction:" << reaction;

    KDL::Frame H_sensor = KdlVectorConverter::vectorToFrame(reaction);
    KDL::Frame H_device = H_device_sensor * H_sensor;

    auto pose = KdlVectorConverter::frameToVector(H_device);

    iCartesianControl->pose(pose);
    return true;
}

bool HapticFeedback::interruptModule()
{
    if (iCartesianControl)
    {
        return iCartesianControl->stopControl();
    }

    return true;
}

bool HapticFeedback::close()
{
    bool ret = true;

    if (cartesianClientDevice.isValid())
    {
        ret &= cartesianClientDevice.close();
        iCartesianControl = nullptr;
    }

    if (sensorClientDevice.isValid())
    {
        ret &= sensorClientDevice.close();
        iSensor = nullptr;
    }

    return ret;
}

double HapticFeedback::getPeriod()
{
    return period;
}

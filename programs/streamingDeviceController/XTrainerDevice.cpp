#include "XTrainerDevice.hpp"

#include <algorithm> // std::clamp, std::copy

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

XTrainerDevice::XTrainerDevice(yarp::os::Searchable & config)
    : StreamingDevice(config)
{
    smoothFactor = config.check("smoothFactor", yarp::os::Value(1.0)).asFloat64();
    smoothFactor = std::clamp(smoothFactor, 0.0, 1.0);

    yCInfo(SDC) << "Using smooth factor:" << smoothFactor;

    kinematics = config.check("kinematics", yarp::os::Value("")).asString();

    yCInfo(SDC) << "Using kinematics file:" << kinematics;

    const auto & v = config.find("deviceFrameRPY");

    if (!v.isNull())
    {
        const auto * deviceFrameRPY = v.asList();

        if (!deviceFrameRPY->isNull() && deviceFrameRPY->size() == 3)
        {
            auto roll = deviceFrameRPY->get(0).asFloat64() * KDL::deg2rad;
            auto pitch = deviceFrameRPY->get(1).asFloat64() * KDL::deg2rad;
            auto yaw = deviceFrameRPY->get(2).asFloat64() * KDL::deg2rad;

            yCInfo(SDC) << "deviceFrameRPY [rad]:" << roll << pitch << yaw;

            H_robot_device = KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw));
            H_device_robot = H_robot_device.Inverse();
        }
    }

    data.resize(7); // although unused, make room for the trigger joint
}

bool XTrainerDevice::acquireInterfaces()
{
    if (!yarp::dev::PolyDriver::view(iJoypadController))
    {
        yCWarning(SDC) << "Could not view IJoypadController interface";
        return false;
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (std::size_t axisCount; !iJoypadController->getAxisCount(axisCount) || axisCount != 7)
#else
    if (unsigned int axisCount; !iJoypadController->getAxisCount(axisCount) || axisCount != 7)
#endif
    {
        yCWarning(SDC) << "Unable to query number of axes or wrong value";
        return false;
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (std::size_t buttonCount; !iJoypadController->getButtonCount(buttonCount) || buttonCount != 2)
#else
    if (unsigned int buttonCount; !iJoypadController->getButtonCount(buttonCount) || buttonCount != 2)
#endif
    {
        yCWarning(SDC) << "Unable to query number of buttons or wrong value";
        return false;
    }

    yarp::os::Property solverOptions = {
        {"device", yarp::os::Value("KdlSolver")},
        {"kinematics", yarp::os::Value(kinematics)},
        // we don't need to solve IK, but otherwise the default solver will complain about missing joint limits
        {"ikPos", yarp::os::Value("nr")}
    };

    if (!solverDevice.open(solverOptions) || !solverDevice.view(iCartesianSolver))
    {
        yCWarning(SDC) << "Unable to open solver device or invalid interface";
        return false;
    }

    return true;
}

bool XTrainerDevice::initialize(const ICartesianControl::config_map_t & params)
{
    if (params.find(ICartesianControl::Config::STREAMING_CMD) != params.end())
    {
        if (auto value = static_cast<yarp::conf::vocab32_t>(ICartesianControl::Streaming::POSE);
            std::get<yarp::conf::vocab32_t>(params.at(ICartesianControl::Config::STREAMING_CMD)) != value &&
            !iCartesianControl->setParameter(ICartesianControl::Config::STREAMING_CMD, value))
        {
            yCWarning(SDC) << "Unable to preset streaming command";
            return false;
        }
    }

    if (params.find(ICartesianControl::Config::FRAME) != params.end())
    {
        if (auto value = static_cast<yarp::conf::vocab32_t>(ICartesianSolver::Frame::BASE);
            std::get<yarp::conf::vocab32_t>(params.at(ICartesianControl::Config::FRAME)) != value &&
            !iCartesianControl->setParameter(ICartesianControl::Config::FRAME, value))
        {
            yCWarning(SDC) << "Unable to set inertial reference frame";
            return false;
        }
    }

    ICartesianControl::ControllerState state;

    if (!iCartesianControl->getState(state))
    {
        yCWarning(SDC) << "getState failed";
        return false;
    }

    yCInfo(SDC) << "Initial TCP offset:"
                << state.x[0] << state.x[1] << state.x[2] << "[m],"
                << state.x[3] << state.x[4] << state.x[5] << "[rad]";

    KDL::Frame H_robot_initial = KdlVectorConverter::vectorToFrame(state.x);

    H_robot_device_initial = H_robot_initial * H_robot_device;

    if (!acquireData())
    {
        yCWarning(SDC) << "Initial acquireData failed";
        return false;
    }

    prevJointData = data;

    std::vector<double> initialDeviceOffset;

    if (!iCartesianSolver->forwardKinematics(data, initialDeviceOffset))
    {
        yCWarning(SDC) << "Initial forwardKinematics failed";
        return false;
    }

    yCInfo(SDC) << "Initial device offset:"
                << initialDeviceOffset[0] << initialDeviceOffset[1] << initialDeviceOffset[2] << "[m],"
                << initialDeviceOffset[3] << initialDeviceOffset[4] << initialDeviceOffset[5] << "[rad]";

    H_device_initial_inv = KdlVectorConverter::vectorToFrame(initialDeviceOffset).Inverse();

    return true;
}

bool XTrainerDevice::acquireData()
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    if (!iJoypadController->getAllAxes(data))
    {
        yCWarning(SDC) << "Unable to acquire data from IJoypadController axes";
        return false;
    }
#else
    for (int i = 0; i < data.size(); i++)
    {
        if (!iJoypadController->getAxis(i, data[i]))
        {
            yCWarning(SDC) << "Unable to acquire data from IJoypadController axis" << i;
            return false;
        }
    }
#endif

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    double button1, button2;
#else
    float button1, button2;
#endif

    if (!iJoypadController->getButton(0, button1) || !iJoypadController->getButton(1, button2))
    {
        yCWarning(SDC) << "Unable to acquire data from IJoypadController buttons";
        return false;
    }

    buttonClose = (button1 != 0.0);
    buttonOpen = (button2 != 0.0);

    yCDebug(SDC) << "stick:" << data << "|| buttons:" << buttonClose << buttonOpen;

    return true;
}

bool XTrainerDevice::transformData(double scaling)
{
    for (int i = 0; i < 6; i++)
    {
        data[i] = smoothFactor * data[i] + (1.0 - smoothFactor) * prevJointData[i];
        prevJointData[i] = data[i];
    }

    if (!iCartesianSolver->forwardKinematics(data, data))
    {
        yCWarning(SDC) << "forwardKinematics failed";
        return false;
    }

    KDL::Frame H_device = KdlVectorConverter::vectorToFrame(data);

    // current pose wrt. the initial pose (device)
    KDL::Frame H_device_diff = H_device_initial_inv * H_device;

    if (hasFixedAxes)
    {
        auto vec = KdlVectorConverter::frameToVector(H_device_diff);

        for (int i = 0; i < 6; i++)
        {
            if (fixedAxes[i])
            {
                vec[i] = 0.0;
            }
        }

        H_device_diff = KdlVectorConverter::vectorToFrame(vec);
    }
    else if (scaling != 1.0)
    {
        H_device_diff.p.x(H_device_diff.p.x() / scaling);
        H_device_diff.p.y(H_device_diff.p.y() / scaling);
        H_device_diff.p.z(H_device_diff.p.z() / scaling);
    }

    // H_robot_device_initial: robot base-to-TCP where TCP follows the device TCP orientation (initial)
    // H_device_diff: difference between current device pose and initial device pose
    // H_device_robot: device TCP to robot TCP (inverse of H_robot_device in order to refer to the robot base)
    KDL::Frame H_robot = H_robot_device_initial * H_device_diff * H_device_robot;

    data = KdlVectorConverter::frameToVector(H_robot);

    return true;
}

ICartesianControl::Actuator XTrainerDevice::getActuatorState()
{
    if (buttonClose)
    {
        actuatorState = ICartesianControl::Actuator::CLOSE;
    }
    else if (buttonOpen)
    {
        actuatorState = ICartesianControl::Actuator::OPEN;
    }
    else if (actuatorState != ICartesianControl::Actuator::NONE)
    {
        if (actuatorState != ICartesianControl::Actuator::STOP)
        {
            actuatorState = ICartesianControl::Actuator::STOP;
        }
        else
        {
            actuatorState = ICartesianControl::Actuator::NONE;
        }
    }
    else
    {
        actuatorState = ICartesianControl::Actuator::NONE;
    }

    return actuatorState;
}

void XTrainerDevice::sendMovementCommand(double timestamp)
{
    iCartesianControl->pose(data);
}

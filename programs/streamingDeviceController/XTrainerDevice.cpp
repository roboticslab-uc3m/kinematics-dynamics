#include "XTrainerDevice.hpp"

#include <algorithm> // std::copy

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

XTrainerDevice::XTrainerDevice(yarp::os::Searchable & config)
    : StreamingDevice(config)
{
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

            H_ee_device = KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw));
            H_device_ee = H_ee_device.Inverse();
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

    initialTcpOffset = state.x;

    yCInfo(SDC) << "Initial TCP offset:"
                << initialTcpOffset[0] << initialTcpOffset[1] << initialTcpOffset[2] << "[m],"
                << initialTcpOffset[3] << initialTcpOffset[4] << initialTcpOffset[5] << "[rad]";

    auto H_base_ee = KdlVectorConverter::vectorToFrame(initialTcpOffset);

    H_base_device = H_base_ee * H_ee_device;

    if (!acquireData())
    {
        yCWarning(SDC) << "Initial acquireData failed";
        return false;
    }

    if (!iCartesianSolver->forwardKinematics(data, initialDeviceOffset))
    {
        yCWarning(SDC) << "Initial forwardKinematics failed";
        return false;
    }

    yCInfo(SDC) << "Initial device offset:"
                << initialDeviceOffset[0] << initialDeviceOffset[1] << initialDeviceOffset[2] << "[m],"
                << initialDeviceOffset[3] << initialDeviceOffset[4] << initialDeviceOffset[5] << "[rad]";

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
    if (!iCartesianSolver->forwardKinematics(data, data))
    {
        yCWarning(SDC) << "forwardKinematics failed";
        return false;
    }

    for (int i = 0; i < 6; i++)
    {
        if (fixedAxes[i])
        {
            data[i] = 0.0;
        }
        else
        {
            data[i] -= initialDeviceOffset[i];

            if (i < 3)
            {
                data[i] /= scaling;
            }
        }
    }

    KDL::Rotation R_device;

    if (!fixedAxes[3] && !fixedAxes[4] && !fixedAxes[5])
    {
        KDL::Vector vec(data[3], data[4], data[5]);
        R_device = KDL::Rotation::Rot(vec, vec.Norm());
    }

    KDL::Vector p_device(data[0], data[1], data[2]);
    KDL::Frame H_device(R_device, p_device);

    // undo device frame rotation with H_device_ee
    KDL::Frame H_base_hand = H_base_device * H_device * H_device_ee;

    data = KdlVectorConverter::frameToVector(H_base_hand);

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

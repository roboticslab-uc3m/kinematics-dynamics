#include "SpnavSensorDevice.hpp"

#include <algorithm> // std::copy

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

SpnavSensorDevice::SpnavSensorDevice(yarp::os::Searchable & config, bool usingPose, double gain)
    : StreamingDevice(config),
        usingPose(usingPose),
        gain(gain)
{}

bool SpnavSensorDevice::acquireInterfaces()
{
    if (!yarp::dev::PolyDriver::view(iJoypadController))
    {
        yCWarning(SDC) << "Could not view IJoypadController interface";
        return false;
    }

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    if (std::size_t stickCount; !iJoypadController->getStickCount(stickCount) || stickCount < 1)
#else
    if (unsigned int stickCount; !iJoypadController->getStickCount(stickCount) || stickCount < 1)
#endif
    {
        yCWarning(SDC) << "Unable to query number of sticks or wrong value";
        return false;
    }

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    if (std::size_t stickDoF; !iJoypadController->getStickDoF(0, stickDoF) || stickDoF < 6)
#else
    if (unsigned int stickDoF; !iJoypadController->getStickDoF(0, stickDoF) || stickDoF < 6)
#endif
    {
        yCWarning(SDC) << "Unable to query number of stick DoF or wrong value";
        return false;
    }

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    if (std::size_t buttonCount; !iJoypadController->getButtonCount(buttonCount) || buttonCount < 2)
#else
    if (unsigned int buttonCount; !iJoypadController->getButtonCount(buttonCount) || buttonCount < 2)
#endif
    {
        yCWarning(SDC) << "Unable to query number of buttons or wrong value";
        return false;
    }

    return true;
}

bool SpnavSensorDevice::initialize(bool usingStreamingPreset)
{
    if (usingPose && gain <= 0.0)
    {
        yCWarning(SDC) << "Invalid gain for pose command:" << gain;
        return false;
    }

    if (usingStreamingPreset)
    {
        auto cmd = usingPose ? ICartesianControl::Streaming::POSE : ICartesianControl::Streaming::TWIST;

        if (!iCartesianControl->setParameter(ICartesianControl::Config::STREAMING_CMD, static_cast<double>(cmd)))
        {
            yCWarning(SDC) << "Unable to preset streaming command";
            return false;
        }
    }

    if (!iCartesianControl->setParameter(ICartesianControl::Config::FRAME, static_cast<double>(ICartesianSolver::Frame::BASE)))
    {
        yCWarning(SDC) << "Unable to set inertial reference frame";
        return false;
    }

    if (usingPose && !iCartesianControl->getState(currentX))
    {
        yCWarning(SDC) << "Unable to stat initial position, assuming zero";
        currentX.resize(6, 0.0);
    }

    return true;
}

bool SpnavSensorDevice::acquireData()
{
#if YARP_VERSION_COMPARE(>=, 4,0,0)
    yarp::dev::StickData stick;
#else
    yarp::sig::Vector stick;
#endif

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    if (!iJoypadController->getStick(0, stick, yarp::dev::IJoypadController::JoypadCtrl_coordinateMode::JypCtrlcoord_CARTESIAN))
#else
    if (!iJoypadController->getStick(0, stick, yarp::dev::IJoypadController::JypCtrlcoord_CARTESIAN))
#endif
    {
        yCWarning(SDC) << "Unable to acquire data from IJoypadController stick";
        return false;
    }

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    double button1, button2;
#else
    float button1, button2;
#endif

    if (!iJoypadController->getButton(0, button1) ||
        !iJoypadController->getButton(1, button2))
    {
        yCWarning(SDC) << "Unable to acquire data from IJoypadController buttons";
        return false;
    }

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    // FIXME: https://github.com/robotology/yarp/issues/3370
    data[0] = stick.s1;
    data[1] = stick.s2;
#else
    std::copy(stick.begin(), stick.begin() + 6, data.begin());
#endif

    buttonClose = (button1 != 0.0);
    buttonOpen = (button2 != 0.0);

    yCDebug(SDC) << "stick:" << stick.toString() << "|| buttons:" << buttonClose << buttonOpen;

    return true;
}

bool SpnavSensorDevice::transformData(double scaling)
{
    if (usingPose)
    {
        for (int i = 0; i < 6; i++)
        {
            if (!fixedAxes[i])
            {
                data[i] = currentX[i] + (gain / scaling) * data[i];
            }
            else
            {
                data[i] = currentX[i];
            }
        }

        return true;
    }
    else
    {
        return StreamingDevice::transformData(scaling);
    }
}

ICartesianControl::Actuator SpnavSensorDevice::getActuatorState()
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

bool SpnavSensorDevice::hasValidMovementData() const
{
    if (usingPose)
    {
        for (int i = 0; i < 6; i++)
        {
            if (!fixedAxes[i] && data[i] != currentX[i])
            {
                return true;
            }
        }

        return false;
    }
    else
    {
        return StreamingDevice::hasValidMovementData();
    }
}

void SpnavSensorDevice::sendMovementCommand(double timestamp)
{
    if (usingPose)
    {
        iCartesianControl->pose(data);

        for (int i = 0; i < 6; i++)
        {
            currentX[i] = data[i];
        }
    }
    else
    {
        iCartesianControl->twist(data);
    }
}

void SpnavSensorDevice::stopMotion()
{
    if (!usingPose)
    {
        std::vector<double> zeros(6, 0.0);
        iCartesianControl->twist(zeros);
    }
}

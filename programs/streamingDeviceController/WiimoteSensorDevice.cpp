#include "WiimoteSensorDevice.hpp"

#include <algorithm> // std::copy

#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_STEP = 0.01;

WiimoteSensorDevice::WiimoteSensorDevice(yarp::os::Searchable & config, bool usingPose)
    : StreamingDevice(config),
      usingPose(usingPose)
{
    data.resize(3); // already called by base constructor
    step = config.check("step", yarp::os::Value(DEFAULT_STEP), "").asFloat64();
}

bool WiimoteSensorDevice::acquireInterfaces()
{
    if (!yarp::dev::PolyDriver::view(iJoypadController))
    {
        yCWarning(SDC) << "Could not view IJoypadController interface";
        return false;
    }

    if (unsigned int axisCount; !iJoypadController->getAxisCount(axisCount) || axisCount < 2)
    {
        yCWarning(SDC) << "Unable to query number of axes or wrong value";
        return false;
    }

    if (unsigned int buttonCount; !iJoypadController->getButtonCount(buttonCount) || buttonCount < 3)
    {
        yCWarning(SDC) << "Unable to query number of buttons or wrong value";
        return false;
    }

    return true;
}

bool WiimoteSensorDevice::initialize(bool usingStreamingPreset)
{
    if (usingPose && step <= 0.0)
    {
        yCWarning(SDC) << "Invalid step:" << step;
        return false;
    }

    if (usingStreamingPreset)
    {
        int cmd = usingPose ? VOCAB_CC_POSE : VOCAB_CC_TWIST;

        if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, cmd))
        {
            yCWarning(SDC) << "Unable to preset streaming command";
            return false;
        }
    }

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::TCP_FRAME))
    {
        yCWarning(SDC) << "Unable to set TCP reference frame";
        return false;
    }

    return true;
}

bool WiimoteSensorDevice::acquireData()
{
    double axis1, axis2;
    float button1, button2, button3;

    if (!iJoypadController->getAxis(0, axis1) ||
        !iJoypadController->getAxis(1, axis2) ||
        !iJoypadController->getButton(0, button1) ||
        !iJoypadController->getButton(1, button2) ||
        !iJoypadController->getButton(2, button3))
    {
        yCWarning(SDC) << "Unable to acquire data from IJoypadController";
        return false;
    }

    data = {axis1, axis2, 0.0};

    buttonA = (button1 != 0.f);
    buttonB = (button2 != 0.f);
    yawActive = (button3 != 0.f);

    yCDebug(SDC) << "axes:" << axis1 << axis2 << "|| buttons:" << button1 << button2 << button3;

    return true;
}

bool WiimoteSensorDevice::transformData(double scaling)
{
    if (buttonA && buttonB)
    {
        mode = ROT;
    }
    else if (buttonA)
    {
        mode = FWD;
    }
    else if (buttonB)
    {
        mode = BKWD;
    }
    else
    {
        mode = NONE;
        return true;
    }

    data[1] = -data[1] / scaling;

    if (yawActive)
    {
        data[2] = data[0] / scaling;
        data[0] = 0.0;
    }
    else
    {
        data[0] /= scaling;
    }

    return true;
}

bool WiimoteSensorDevice::hasValidMovementData() const
{
    return mode != NONE;
}

void WiimoteSensorDevice::sendMovementCommand(double timestamp)
{
    std::vector<double> xdot(6, 0.0);
    std::copy(data.begin(), data.end(), xdot.begin() + 3);

    switch (mode)
    {
    case FWD:
        xdot[2] = step;
        break;
    case BKWD:
        xdot[2] = -step;
        break;
    case ROT:
        xdot[2] = 0.0; // for the sake of explicitness
        break;
    default:
        return;
    }

    if (usingPose)
    {
        iCartesianControl->pose(xdot);
    }
    else
    {
        iCartesianControl->twist(xdot);
    }
}

void WiimoteSensorDevice::stopMotion()
{
    if (!usingPose)
    {
        std::vector<double> zeros(6, 0.0);
        iCartesianControl->twist(zeros);
    }
}

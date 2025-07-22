#include "SpnavSensorDevice.hpp"

#include <algorithm> // std::copy

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

    if (unsigned int stickCount; !iJoypadController->getStickCount(stickCount) || stickCount < 1)
    {
        yCWarning(SDC) << "Unable to query number of sticks or wrong value";
        return false;
    }

    if (unsigned int stickDoF; !iJoypadController->getStickDoF(0, stickDoF) || stickDoF < 6)
    {
        yCWarning(SDC) << "Unable to query number of stick DoF or wrong value";
        return false;
    }

    if (unsigned int buttonCount; !iJoypadController->getButtonCount(buttonCount) || buttonCount < 2)
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
        int cmd = usingPose ? VOCAB_CC_POSE : VOCAB_CC_TWIST;

        if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, cmd))
        {
            yCWarning(SDC) << "Unable to preset streaming command";
            return false;
        }
    }

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::BASE_FRAME))
    {
        yCWarning(SDC) << "Unable to set inertial reference frame";
        return false;
    }

    if (usingPose && !iCartesianControl->stat(currentX))
    {
        yCWarning(SDC) << "Unable to stat initial position, assuming zero";
        currentX.resize(6, 0.0);
    }

    return true;
}

bool SpnavSensorDevice::acquireData()
{
    yarp::sig::Vector stick;

    if (!iJoypadController->getStick(0, stick, yarp::dev::IJoypadController::JypCtrlcoord_CARTESIAN))
    {
        yCWarning(SDC) << "Unable to acquire data from IJoypadController stick";
        return false;
    }

    float button1, button2;

    if (!iJoypadController->getButton(0, button1) ||
        !iJoypadController->getButton(1, button2))
    {
        yCWarning(SDC) << "Unable to acquire data from IJoypadController buttons";
        return false;
    }

    std::copy(stick.begin(), stick.begin() + 6, data.begin());

    buttonClose = (button1 != 0.f);
    buttonOpen = (button2 != 0.f);

    yCDebug(SDC) << "stick:" << stick.toString(4, 1) << "|| buttons:" << buttonClose << buttonOpen;

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

int SpnavSensorDevice::getActuatorState()
{
    if (buttonClose)
    {
        actuatorState = VOCAB_CC_ACTUATOR_CLOSE_GRIPPER;
    }
    else if (buttonOpen)
    {
        actuatorState = VOCAB_CC_ACTUATOR_OPEN_GRIPPER;
    }
    else if (actuatorState != VOCAB_CC_ACTUATOR_NONE)
    {
        if (actuatorState != VOCAB_CC_ACTUATOR_STOP_GRIPPER)
        {
            actuatorState = VOCAB_CC_ACTUATOR_STOP_GRIPPER;
        }
        else
        {
            actuatorState = VOCAB_CC_ACTUATOR_NONE;
        }
    }
    else
    {
        actuatorState = VOCAB_CC_ACTUATOR_NONE;
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

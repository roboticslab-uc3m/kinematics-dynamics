#include "SpnavSensorDevice.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include "LogComponent.hpp"

using namespace roboticslab;

SpnavSensorDevice::SpnavSensorDevice(yarp::os::Searchable & config, bool usingPose, double gain)
    : StreamingDevice(config),
      iAnalogSensor(nullptr),
      usingPose(usingPose),
      gain(gain),
      buttonClose(false),
      buttonOpen(false)
{}

bool SpnavSensorDevice::acquireInterfaces()
{
    bool ok = true;

    if (!yarp::dev::PolyDriver::view(iAnalogSensor))
    {
        yCWarning(SDC) << "Could not view iAnalogSensor";
        ok = false;
    }

    return ok;
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
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    yCDebug(SDC) << data.toString(4, 1);

    if (data.size() != 6 && data.size() != 8)
    {
        yCWarning(SDC) << "Invalid data size:" << data.size();
        return false;
    }

    for (int i = 0; i < 6; i++)
    {
        this->data[i] = data[i];
    }

    if (data.size() == 8)
    {
        buttonClose = data[6] == 1;
        buttonOpen = data[7] == 1;
    }

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

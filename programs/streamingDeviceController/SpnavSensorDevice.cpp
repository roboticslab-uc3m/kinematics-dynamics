#include "SpnavSensorDevice.hpp"

#include <yarp/sig/Vector.h>

#include <ColorDebug.h>

roboticslab::SpnavSensorDevice::SpnavSensorDevice(yarp::os::Searchable & config, bool usingMovi, double gain)
    : StreamingDevice(config),
      iAnalogSensor(NULL),
      usingMovi(usingMovi),
      gain(gain)
{}

bool roboticslab::SpnavSensorDevice::acquireInterfaces()
{
    bool ok = true;

    if (!PolyDriver::view(iAnalogSensor))
    {
        CD_WARNING("Could not view iAnalogSensor.\n");
        ok = false;
    }

    return ok;
}

bool roboticslab::SpnavSensorDevice::initialize(bool usingStreamingPreset)
{
    if (usingMovi && gain <= 0.0)
    {
        CD_WARNING("Invalid gain for movi command: %f.\n", gain);
        return false;
    }

    if (usingStreamingPreset)
    {
        int cmd = usingMovi ? VOCAB_CC_MOVI : VOCAB_CC_TWIST;

        if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, cmd))
        {
            CD_WARNING("Unable to preset streaming command.\n");
            return false;
        }
    }

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::BASE_FRAME))
    {
        CD_WARNING("Unable to set inertial reference frame.\n");
        return false;
    }

    if (!iCartesianControl->stat(currentX))
    {
        CD_WARNING("Unable to stat initial position.\n");
        return false;
    }

    return true;
}

bool roboticslab::SpnavSensorDevice::acquireData()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 6 && data.size() != 8)
    {
        CD_WARNING("Invalid data size: %zu.\n", data.size());
        return false;
    }

    for (int i = 0; i < data.size(); i++)
    {
        this->data[i] = data[i];
    }

    return true;
}

bool roboticslab::SpnavSensorDevice::transformData(double scaling)
{
    if (usingMovi)
    {
        for (int i = 0; i < data.size(); i++)
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

int roboticslab::SpnavSensorDevice::getActuatorState()
{
    int button1 = data[6];
    int button2 = data[7];

    if (button1 == 1)
    {
        actuatorState = VOCAB_CC_ACTUATOR_CLOSE_GRIPPER;
    }
    else if (button2 == 1)
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

bool roboticslab::SpnavSensorDevice::hasValidMovementData() const
{
    if (usingMovi)
    {
        for (int i = 0; i < data.size(); i++)
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

void roboticslab::SpnavSensorDevice::sendMovementCommand()
{
    if (usingMovi)
    {
        iCartesianControl->movi(data);

        for (int i = 0; i < data.size(); i++)
        {
            currentX[i] = data[i];
        }
    }
    else
    {
        iCartesianControl->twist(data);
    }
}

void roboticslab::SpnavSensorDevice::stopMotion()
{
    if (!usingMovi)
    {
        std::vector<double> zeros(6, 0.0);
        iCartesianControl->twist(zeros);
    }
}

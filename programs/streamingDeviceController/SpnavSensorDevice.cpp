#include "SpnavSensorDevice.hpp"

#include <yarp/sig/Vector.h>

#include <ColorDebug.hpp>

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

bool roboticslab::SpnavSensorDevice::acquireData()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 8)
    {
        CD_WARNING("Invalid data size: %d.\n", data.size());
        return false;
    }

    for (int i = 0; i < data.size(); i++)
    {
        this->data[i] = data[i];
    }

    return true;
}

int roboticslab::SpnavSensorDevice::getActuatorState()
{
    int button1 = data[6];
    int button2 = data[7];

    if (button1 == 1)
    {
        actuatorState = 1;
    }
    else if (button2 == 1)
    {
        actuatorState = 2;
    }
    else
    {
        if (actuatorState != 0)
        {
            actuatorState = 3;
        }

        actuatorState = 0;
    }

    return actuatorState;
}

void roboticslab::SpnavSensorDevice::sendMovementCommand()
{
    iCartesianControl->vmos(data);
}

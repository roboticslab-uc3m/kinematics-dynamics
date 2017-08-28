#include "SpnavSensorDevice.hpp"

#include <yarp/sig/Vector.h>

#include <ColorDebug.hpp>

bool roboticslab::SpnavSensorDevice::acquireInterfaces()
{
    if (!PolyDriver::view(iAnalogSensor))
    {
        CD_ERROR("Could not view iAnalogSensor.\n");
        return false;
    }

    return true;
}

bool roboticslab::SpnavSensorDevice::acquireData()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 6)
    {
        CD_ERROR("Invalid data size: %d.\n", data.size());
        return false;
    }

    for (int i = 0; i < data.size(); i++)
    {
        this->data[i] = data[i];
    }

    return true;
}

bool roboticslab::SpnavSensorDevice::sendMovementCommand()
{
    return iCartesianControl->vmos(data);
}

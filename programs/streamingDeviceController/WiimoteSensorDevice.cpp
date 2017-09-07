#include "WiimoteSensorDevice.hpp"

#include <cmath>

#include <yarp/sig/Vector.h>

#include <ColorDebug.hpp>

bool roboticslab::WiimoteSensorDevice::acquireInterfaces()
{
    bool ok = true;

    if (!PolyDriver::view(iAnalogSensor))
    {
        CD_WARNING("Could not view iAnalogSensor.\n");
        ok = false;
    }

    return ok;
}

bool roboticslab::WiimoteSensorDevice::acquireData()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 5)
    {
        CD_WARNING("Invalid data size: %d.\n", data.size());
        return false;
    }

    for (int i = 0; i < data.size(); i++)
    {
        buffer[i] = data[i];
    }

    return true;
}

bool roboticslab::WiimoteSensorDevice::transformData(double scaling)
{
    bool buttonA = buffer[2] == 1.0;
    bool buttonB = buffer[3] == 1.0;
    bool yawActive = buffer[4] == 1.0;

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

    data[1] = -buffer[1] / scaling;

    if (yawActive)
    {
        data[0] = 0.0;
        data[2] = buffer[0] / scaling;
    }
    else
    {
        data[0] = buffer[0] / scaling;
        data[2] = 0.0;
    }

    return true;
}

bool roboticslab::WiimoteSensorDevice::hasValidMovementData() const
{
    return mode != NONE;
}

void roboticslab::WiimoteSensorDevice::sendMovementCommand()
{
    switch (mode)
    {
    case FWD:
        iCartesianControl->fwd(data, step);
        break;
    case BKWD:
        iCartesianControl->bkwd(data, step);
        break;
    case ROT:
        iCartesianControl->rot(data);
        break;
    default:
        return;
    }
}

#include "WiimoteSensorDevice.hpp"

#include <cmath>
#include <algorithm>

#include <yarp/sig/Vector.h>

#include <ColorDebug.h>

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

bool roboticslab::WiimoteSensorDevice::initialize(bool usingStreamingPreset)
{
    if (usingStreamingPreset && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING, VOCAB_CC_TWIST))
    {
        CD_WARNING("Unable to preset streaming command.\n");
        return false;
    }

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::TCP_FRAME))
    {
        CD_WARNING("Unable to set TCP reference frame.\n");
        return false;
    }

    return true;
}

bool roboticslab::WiimoteSensorDevice::acquireData()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 5)
    {
        CD_WARNING("Invalid data size: %zu.\n", data.size());
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

    iCartesianControl->twist(xdot);
}

void roboticslab::WiimoteSensorDevice::stopMotion()
{
    std::vector<double> zeros(6, 0.0);
    iCartesianControl->twist(zeros);
}

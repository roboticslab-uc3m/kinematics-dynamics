#include "SpnavSensorDevice.hpp"

#include <yarp/sig/Vector.h>

#include <ColorDebug.h>

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
    if (usingStreamingPreset && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING, VOCAB_CC_TWIST))
    {
        CD_WARNING("Unable to preset streaming command.\n");
        return false;
    }

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::BASE_FRAME))
    {
        CD_WARNING("Unable to set inertial reference frame.\n");
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
        CD_WARNING("Invalid data size: %zu.\n", data.size());
        return false;
    }

    for (int i = 0; i < data.size(); i++)
    {
        this->data[i] = data[i];
    }

    return true;
}

void roboticslab::SpnavSensorDevice::sendMovementCommand()
{
    iCartesianControl->twist(data);
}

void roboticslab::SpnavSensorDevice::stopMotion()
{
    std::vector<double> zeros(6, 0.0);
    iCartesianControl->twist(zeros);
}

#include "LeapMotionSensorDevice.hpp"

#include <cmath>

#include <yarp/sig/Vector.h>

#include <ColorDebug.hpp>

bool roboticslab::LeapMotionSensorDevice::acquireInterfaces()
{
    bool ok = true;

    if (!PolyDriver::view(iAnalogSensor))
    {
        CD_WARNING("Could not view iAnalogSensor.\n");
        ok = false;
    }

    return ok;
}

bool roboticslab::LeapMotionSensorDevice::initialize()
{
    initialOffset.resize(3);

    int state;
    std::vector<double> x;

    if (!iCartesianControl->stat(state, x))
    {
        CD_WARNING("stat failed.\n");
        return false;
    }

    initialOffset[0] = x[0];
    initialOffset[1] = x[1];
    initialOffset[2] = x[2];

    CD_INFO("Initial offset [m]: %f %f %f\n", initialOffset[0], initialOffset[1], initialOffset[2]);

    return true;
}

bool roboticslab::LeapMotionSensorDevice::acquireData()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 6)
    {
        CD_WARNING("Invalid data size: %d.\n", data.size());
        return false;
    }

    // convert to meters
    this->data[0] = data[0] * 0.01;
    this->data[1] = data[1] * 0.01;
    this->data[2] = data[2] * 0.01;

    // convert to radians
    this->data[3] = data[3] * M_PI / 180.0;
    this->data[4] = (data[4] + 90.0) * M_PI / 180.0;
    this->data[5] = data[5] * M_PI / 180.0;

    return true;
}

bool roboticslab::LeapMotionSensorDevice::transformData(double scaling)
{
    data[2] -= VERTICAL_OFFSET;

    for (int i = 0; i < 3; i++)
    {
        data[i] /= scaling;
        data[i] += initialOffset[i];
    }

    return true;
}

void roboticslab::LeapMotionSensorDevice::sendMovementCommand()
{
    iCartesianControl->pose(data, period);
}

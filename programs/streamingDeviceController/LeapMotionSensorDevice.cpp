#include "LeapMotionSensorDevice.hpp"

#include <cmath>

#include <yarp/sig/Vector.h>

#include <kdl/frames.hpp>

#include <ColorDebug.hpp>

namespace
{
    KDL::Frame2 frame;
}

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
    int state;
    std::vector<double> x;

    if (!iCartesianControl->stat(state, initialOffset))
    {
        CD_WARNING("stat failed.\n");
        return false;
    }

    CD_INFO("Initial offset: %f %f %f [m], %f %f %f [rad]\n",
            initialOffset[0], initialOffset[1], initialOffset[2],
            initialOffset[3], initialOffset[4], initialOffset[5]);

    const KDL::Rotation2 rot(std::atan2(initialOffset[1], initialOffset[0]));
    const KDL::Vector2 vec(initialOffset[0], initialOffset[1]);

    frame = KDL::Frame2(rot, vec);

    return true;
}

bool roboticslab::LeapMotionSensorDevice::acquireData()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() < 6)
    {
        CD_WARNING("Invalid data size: %zu.\n", data.size());
        return false;
    }

    // convert to meters
    this->data[0] = -data[2] * 0.001;
    this->data[1] = -data[0] * 0.001;
    this->data[2] = data[1] * 0.001;

    // keep in radians
    this->data[3] = initialOffset[3];
    this->data[4] = initialOffset[4];
    this->data[5] = initialOffset[5];

    return true;
}

bool roboticslab::LeapMotionSensorDevice::transformData(double scaling)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] /= scaling;
    }

    KDL::Vector2 vec_leap(data[0], data[1]);
    KDL::Vector2 vec_base = frame * vec_leap;

    data[0] = vec_base.x();
    data[1] = vec_base.y();
    data[2] += initialOffset[2] - VERTICAL_OFFSET;

    return true;
}

void roboticslab::LeapMotionSensorDevice::sendMovementCommand()
{
    iCartesianControl->pose(data, period);
}

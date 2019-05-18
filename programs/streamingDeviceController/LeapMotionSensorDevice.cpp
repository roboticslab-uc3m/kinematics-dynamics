#include "LeapMotionSensorDevice.hpp"

#include <cmath>

#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

#include <kdl/frames.hpp>

#include <KdlVectorConverter.hpp>

#include <ColorDebug.h>

namespace
{
    KDL::Frame frame_base_leap, frame_ee_leap, frame_leap_ee;
}

roboticslab::LeapMotionSensorDevice::LeapMotionSensorDevice(yarp::os::Searchable & config, double period)
    : StreamingDevice(config),
      iAnalogSensor(NULL),
      period(period)
{
    yarp::os::Bottle *leapFrameRPY = config.find("leapFrameRPY").asList();

    if (!leapFrameRPY->isNull() && leapFrameRPY->size() == 3)
    {
        double roll = leapFrameRPY->get(0).asFloat64() * M_PI / 180.0;
        double pitch = leapFrameRPY->get(1).asFloat64() * M_PI / 180.0;
        double yaw = leapFrameRPY->get(2).asFloat64() * M_PI / 180.0;

        CD_INFO("leapFrameRPY [rad]: %f %f %f\n", roll, pitch, yaw);

        frame_ee_leap = KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw));
        frame_leap_ee = frame_ee_leap.Inverse();
    }
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

bool roboticslab::LeapMotionSensorDevice::initialize(bool usingStreamingPreset)
{
    if (usingStreamingPreset && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING, VOCAB_CC_POSE))
    {
        CD_WARNING("Unable to preset streaming command.\n");
        return false;
    }

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::BASE_FRAME))
    {
        CD_WARNING("Unable to set inertial reference frame.\n");
        return false;
    }

    std::vector<double> x;

    if (!iCartesianControl->stat(initialOffset))
    {
        CD_WARNING("stat failed.\n");
        return false;
    }

    CD_INFO("Initial offset: %f %f %f [m], %f %f %f [rad]\n",
            initialOffset[0], initialOffset[1], initialOffset[2],
            initialOffset[3], initialOffset[4], initialOffset[5]);

    KDL::Frame frame_base_ee = KdlVectorConverter::vectorToFrame(initialOffset);

    frame_base_leap = frame_base_ee * frame_ee_leap;

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
    this->data[0] = data[0] * 0.001;
    this->data[1] = data[1] * 0.001;
    this->data[2] = data[2] * 0.001;

    // keep in radians
    this->data[3] = data[3];
    this->data[4] = data[4];
    this->data[5] = data[5];

    return true;
}

bool roboticslab::LeapMotionSensorDevice::transformData(double scaling)
{
    data[1] -= VERTICAL_OFFSET;

    for (int i = 0; i < 6; i++)
    {
        if (fixedAxes[i])
        {
            data[i] = 0.0;
        }
        else if (i < 3)
        {
            data[i] /= scaling;
        }
    }

    KDL::Rotation rot_leap_hand;

    if (!fixedAxes[3] && !fixedAxes[4] && !fixedAxes[5])
    {
        rot_leap_hand = KDL::Rotation::RPY(data[3], data[4], data[5]);
    }

    KDL::Vector vec_leap_hand(data[0], data[1], data[2]);
    KDL::Frame frame_leap_hand(rot_leap_hand, vec_leap_hand);

    // undo LM frame rotation with frame_leap_ee
    KDL::Frame frame_base_hand = frame_base_leap * frame_leap_hand * frame_leap_ee;

    data = KdlVectorConverter::frameToVector(frame_base_hand);

    return true;
}

void roboticslab::LeapMotionSensorDevice::sendMovementCommand()
{
    iCartesianControl->pose(data, period);
}

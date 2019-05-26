#include "LeapMotionSensorDevice.hpp"

#include <cmath>

#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>

#include <kdl/frames.hpp>

#include <KdlVectorConverter.hpp>

#include <ColorDebug.h>

namespace
{
    KDL::Frame frame_base_leap, frame_ee_leap, frame_leap_ee;
}

roboticslab::LeapMotionSensorDevice::LeapMotionSensorDevice(yarp::os::Searchable & config, bool usingMovi, double period)
    : StreamingDevice(config),
      iAnalogSensor(NULL),
      period(period),
      usingMovi(usingMovi)
{
    yarp::os::Value v = config.find("leapFrameRPY");

    if (!v.isNull())
    {
        yarp::os::Bottle *leapFrameRPY = v.asList();

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
    if (!usingMovi && period <= 0.0)
    {
        CD_WARNING("Invalid period for pose command: %f.\n", period);
        return false;
    }

    if (usingStreamingPreset)
    {
        int cmd = usingMovi ? VOCAB_CC_MOVI : VOCAB_CC_POSE;

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

    if (!iCartesianControl->stat(initialTcpOffset))
    {
        CD_WARNING("stat failed.\n");
        return false;
    }

    CD_INFO("Initial TCP offset: %f %f %f [m], %f %f %f [rad]\n",
            initialTcpOffset[0], initialTcpOffset[1], initialTcpOffset[2],
            initialTcpOffset[3], initialTcpOffset[4], initialTcpOffset[5]);

    KDL::Frame frame_base_ee = KdlVectorConverter::vectorToFrame(initialTcpOffset);

    frame_base_leap = frame_base_ee * frame_ee_leap;

    if (!acquireData())
    {
        CD_WARNING("Initial acquireData failed.\n");
        return false;
    }

    initialLeapOffset = data;

    CD_INFO("Initial Leap offset: %f %f %f [m], %f %f %f [rad]\n",
            initialLeapOffset[0], initialLeapOffset[1], initialLeapOffset[2],
            initialLeapOffset[3], initialLeapOffset[4], initialLeapOffset[5]);

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
    for (int i = 0; i < 6; i++)
    {
        if (fixedAxes[i])
        {
            data[i] = 0.0;
        }
        else
        {
            data[i] -= initialLeapOffset[i];

            if (i < 3)
            {
                data[i] /= scaling;
            }
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
    if (usingMovi)
    {
        iCartesianControl->movi(data);
    }
    else
    {
        iCartesianControl->pose(data, period);
    }
}

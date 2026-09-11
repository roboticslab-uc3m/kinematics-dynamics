#include "LeapMotionSensorDevice.hpp"

#include <cmath>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>

#include <kdl/frames.hpp>

#include <KdlVectorConverter.hpp>

#include "LogComponent.hpp"

using namespace roboticslab;

LeapMotionSensorDevice::LeapMotionSensorDevice(yarp::os::Searchable & config, bool usingPose)
    : StreamingDevice(config),
      iAnalogSensor(nullptr),
      usingPose(usingPose),
      previousTimestamp(0.0),
      hasActuator(false),
      grab(false), pinch(false)
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

            yCInfo(SDC) << "leapFrameRPY [rad]:" << roll << pitch << yaw;

            frame_ee_leap = KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw));
            frame_leap_ee = frame_ee_leap.Inverse();
        }
    }
}

bool LeapMotionSensorDevice::acquireInterfaces()
{
    bool ok = true;

    if (!yarp::dev::PolyDriver::view(iAnalogSensor))
    {
        yCWarning(SDC) << "Could not view iAnalogSensor";
        ok = false;
    }

    return ok;
}

bool LeapMotionSensorDevice::initialize(const ICartesianControl::config_map_t & params)
{
    if (params.find(ICartesianControl::Config::STREAMING_CMD) != params.end())
    {
        auto cmd = usingPose ? ICartesianControl::Streaming::POSE : ICartesianControl::Streaming::TWIST;

        if (auto value = static_cast<yarp::conf::vocab32_t>(cmd);
            std::get<yarp::conf::vocab32_t>(params.at(ICartesianControl::Config::STREAMING_CMD)) != value &&
            !iCartesianControl->setParameter(ICartesianControl::Config::STREAMING_CMD, value))
        {
            yCWarning(SDC) << "Unable to preset streaming command";
            return false;
        }
    }

    if (params.find(ICartesianControl::Config::FRAME) != params.end())
    {
        if (auto value = static_cast<yarp::conf::vocab32_t>(ICartesianSolver::Frame::BASE);
            std::get<yarp::conf::vocab32_t>(params.at(ICartesianControl::Config::FRAME)) != value &&
            !iCartesianControl->setParameter(ICartesianControl::Config::FRAME, value))
        {
            yCWarning(SDC) << "Unable to set inertial reference frame";
            return false;
        }
    }

    ICartesianControl::ControllerState state;

    if (!iCartesianControl->getState(state))
    {
        yCWarning(SDC) << "getState failed";
        return false;
    }

    initialTcpOffset = state.x;

    yCInfo(SDC, "Initial TCP offset: %f %f %f [m], %f %f %f [rad]",
           initialTcpOffset[0], initialTcpOffset[1], initialTcpOffset[2],
           initialTcpOffset[3], initialTcpOffset[4], initialTcpOffset[5]);

    KDL::Frame frame_base_ee = KdlVectorConverter::vectorToFrame(initialTcpOffset);

    frame_base_leap = frame_base_ee * frame_ee_leap;

    if (!acquireData())
    {
        yCWarning(SDC) << "Initial acquireData failed";
        return false;
    }

    initialLeapOffset = data;

    yCInfo(SDC, "Initial Leap offset: %f %f %f [m], %f %f %f [rad]",
           initialLeapOffset[0], initialLeapOffset[1], initialLeapOffset[2],
           initialLeapOffset[3], initialLeapOffset[4], initialLeapOffset[5]);

    return true;
}

bool LeapMotionSensorDevice::acquireData()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    yCDebug(SDC) << data.toString(4, 1);

    if (data.size() != 6 && data.size() != 8)
    {
        yCWarning(SDC) << "Invalid data size:" << data.size();
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

    if (data.size() == 8)
    {
        hasActuator = true;

        grab = data[6] == 1.0;
        pinch = data[7] == 1.0;
    }

    return true;
}

bool LeapMotionSensorDevice::transformData(double scaling)
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

ICartesianControl::Actuator LeapMotionSensorDevice::getActuatorState()
{
    if (!hasActuator)
    {
        return ICartesianControl::Actuator::NONE;
    }

    if (grab)
    {
        actuatorState = ICartesianControl::Actuator::CLOSE;
    }
    else if (pinch)
    {
        actuatorState = ICartesianControl::Actuator::OPEN;
    }
    else if (actuatorState != ICartesianControl::Actuator::NONE)
    {
            if (actuatorState != ICartesianControl::Actuator::STOP)
            {
                actuatorState = ICartesianControl::Actuator::STOP;
            }
            else
            {
                actuatorState = ICartesianControl::Actuator::NONE;
            }
    }
    else
    {
        actuatorState = ICartesianControl::Actuator::NONE;
    }

    return actuatorState;
}

void LeapMotionSensorDevice::sendMovementCommand(double timestamp)
{
    if (usingPose)
    {
        iCartesianControl->pose(data);
    }
    else
    {
        KDL::Frame currentPose = KdlVectorConverter::vectorToFrame(data);

        if (previousTimestamp != 0.0) // skip motion for the very first time
        {
            KDL::Twist xdot = KDL::diff(previousPose, currentPose, timestamp - previousTimestamp);
            iCartesianControl->twist(KdlVectorConverter::twistToVector(xdot));
        }

        previousPose = currentPose;
        previousTimestamp = timestamp;
    }
}

#ifndef __LEAP_MOTION_SENSOR_DEVICE_HPP__
#define __LEAP_MOTION_SENSOR_DEVICE_HPP__

#include "StreamingDevice.hpp"

#include <vector>

#include <kdl/frames.hpp>

#include <yarp/dev/IAnalogSensor.h>

namespace roboticslab
{

/**
 * @ingroup streamingDeviceController
 *
 * @brief Represents a LeapMotion device wrapped as an
 * analog sensor by YARP.
 */
class LeapMotionSensorDevice : public StreamingDevice
{
public:
    //! Constructor
    LeapMotionSensorDevice(yarp::os::Searchable & config, bool usingMovi);

    bool acquireInterfaces() override;

    bool initialize(bool usingStreamingPreset) override;

    bool acquireData() override;

    bool transformData(double scaling) override;

    int getActuatorState() override;

    void sendMovementCommand(double timestamp) override;

    void stopMotion() override
    {}

private:
    yarp::dev::IAnalogSensor * iAnalogSensor;

    bool usingMovi;

    std::vector<double> initialTcpOffset;
    std::vector<double> initialLeapOffset;

    KDL::Frame frame_base_leap, frame_ee_leap, frame_leap_ee;

    KDL::Frame previousPose;
    double previousTimestamp;

    bool hasActuator;
    bool grab, pinch;
};

} // namespace roboticslab

#endif // __LEAP_MOTION_SENSOR_DEVICE_HPP__

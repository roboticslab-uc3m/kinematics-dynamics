#ifndef __LEAP_MOTION_SENSOR_DEVICE_HPP__
#define __LEAP_MOTION_SENSOR_DEVICE_HPP__

#include "StreamingDevice.hpp"

#include <vector>

#include <yarp/dev/IAnalogSensor.h>

#define VERTICAL_OFFSET 0.2 // [m]

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
    LeapMotionSensorDevice(yarp::os::Searchable & config, double period)
        : StreamingDevice(config),
          iAnalogSensor(NULL),
          period(period)
    {}

    virtual bool acquireInterfaces();

    virtual bool initialize();

    virtual bool acquireData();

    virtual bool transformData(double scaling);

    virtual void sendMovementCommand();

private:

    yarp::dev::IAnalogSensor * iAnalogSensor;

    double period;

    std::vector<double> initialOffset;
};

}  // namespace roboticslab

#endif  // __LEAP_MOTION_SENSOR_DEVICE_HPP__

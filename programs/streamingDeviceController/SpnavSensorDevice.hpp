#ifndef __SPNAV_SENSOR_DEVICE_HPP__
#define __SPNAV_SENSOR_DEVICE_HPP__

#include "StreamingDevice.hpp"

#include <yarp/dev/IAnalogSensor.h>

namespace roboticslab
{

/**
 * @ingroup streamingDeviceController
 *
 * @brief Represents a spacenav-compatible device, like the SpaceNavigator
 * 6-DOF mouse from 3Dconnexion.
 */
class SpnavSensorDevice : public StreamingDevice
{
public:

    //! Constructor
    SpnavSensorDevice(yarp::os::Searchable & config)
        : StreamingDevice(config),
          iAnalogSensor(NULL)
    {}

    virtual bool acquireInterfaces();

    virtual bool initialize(bool usingStreamingPreset);

    virtual bool acquireData();

    virtual void sendMovementCommand();

    virtual void stopMotion();

private:

    yarp::dev::IAnalogSensor * iAnalogSensor;
};

}  // namespace roboticslab

#endif  // __SPNAV_SENSOR_DEVICE_HPP__

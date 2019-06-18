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
    SpnavSensorDevice(yarp::os::Searchable & config, bool usingMovi, double gain = 0.0);

    virtual bool acquireInterfaces();

    virtual bool initialize(bool usingStreamingPreset);

    virtual bool acquireData();

    virtual bool transformData(double scaling);

    virtual int getActuatorState();

    virtual bool hasValidMovementData() const;

    virtual void sendMovementCommand();

    virtual void stopMotion();

private:

    yarp::dev::IAnalogSensor * iAnalogSensor;

    std::vector<double> currentX;

    bool usingMovi;
    double gain;
};

}  // namespace roboticslab

#endif  // __SPNAV_SENSOR_DEVICE_HPP__

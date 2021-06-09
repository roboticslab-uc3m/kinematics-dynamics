#ifndef __WIIMOTE_SENSOR_DEVICE_HPP__
#define __WIIMOTE_SENSOR_DEVICE_HPP__

#include "StreamingDevice.hpp"

#include <vector>

#include <yarp/os/Value.h>
#include <yarp/dev/IAnalogSensor.h>

#define DEFAULT_STEP 0.01

namespace roboticslab
{

/**
 * @ingroup streamingDeviceController
 *
 * @brief Represents a Wiimote device wrapped as an
 * analog sensor by YARP.
 */
class WiimoteSensorDevice : public StreamingDevice
{
public:
    //! Constructor
    WiimoteSensorDevice(yarp::os::Searchable & config, bool usingMovi);

    bool acquireInterfaces() override;

    bool initialize(bool usingStreamingPreset) override;

    bool acquireData() override;

    bool transformData(double scaling) override;

    bool hasValidMovementData() const override;

    void sendMovementCommand(double timestamp) override;

    void stopMotion() override;

private:
    enum cmd_mode { NONE, FWD, BKWD, ROT };

    yarp::dev::IAnalogSensor * iAnalogSensor;

    cmd_mode mode;

    std::vector<double> buffer;

    bool usingMovi;
    double step;
};

} // namespace roboticslab

#endif // __WIIMOTE_SENSOR_DEVICE_HPP__

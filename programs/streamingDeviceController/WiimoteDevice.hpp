#ifndef __WIIMOTE_SENSOR_DEVICE_HPP__
#define __WIIMOTE_SENSOR_DEVICE_HPP__

#include "StreamingDevice.hpp"

#include <yarp/os/Value.h>
#include <yarp/dev/IJoypadController.h>

namespace roboticslab
{

/**
 * @ingroup streamingDeviceController
 *
 * @brief Represents a Wiimote device wrapped as an
 * analog sensor by YARP.
 */
class WiimoteDevice : public StreamingDevice
{
public:
    WiimoteDevice(yarp::os::Searchable & config, bool usingPose);

    bool acquireInterfaces() override;
    bool initialize(bool usingStreamingPreset) override;
    bool acquireData() override;
    bool transformData(double scaling) override;
    bool hasValidMovementData() const override;
    void sendMovementCommand(double timestamp) override;
    void stopMotion() override;

private:
    enum cmd_mode { NONE, FWD, BKWD, ROT };

    yarp::dev::IJoypadController * iJoypadController {nullptr};
    cmd_mode mode {NONE};
    bool usingPose {false};
    double step {0.0};
    bool buttonA {false};
    bool buttonB {false};
    bool yawActive {false};
};

} // namespace roboticslab

#endif // __WIIMOTE_SENSOR_DEVICE_HPP__

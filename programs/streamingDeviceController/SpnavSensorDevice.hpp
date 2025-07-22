#ifndef __SPNAV_SENSOR_DEVICE_HPP__
#define __SPNAV_SENSOR_DEVICE_HPP__

#include "StreamingDevice.hpp"

#include <yarp/dev/IJoypadController.h>

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
    SpnavSensorDevice(yarp::os::Searchable & config, bool usingPose, double gain = 0.0);

    bool acquireInterfaces() override;
    bool initialize(bool usingStreamingPreset) override;
    bool acquireData() override;
    bool transformData(double scaling) override;
    int getActuatorState() override;
    bool hasValidMovementData() const override;
    void sendMovementCommand(double timestamp) override;
    void stopMotion() override;

private:
    yarp::dev::IJoypadController * iJoypadController {nullptr};
    std::vector<double> currentX;
    bool usingPose {false};
    double gain {0.0};
    bool buttonClose {false};
    bool buttonOpen {false};
};

} // namespace roboticslab

#endif // __SPNAV_SENSOR_DEVICE_HPP__

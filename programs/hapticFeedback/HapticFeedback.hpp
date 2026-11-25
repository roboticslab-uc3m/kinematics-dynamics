#ifndef __HAPTIC_FEEDBACK_HPP__
#define __HAPTIC_FEEDBACK_HPP__

#include <yarp/os/RFModule.h>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include "ICartesianControl.h"

namespace roboticslab
{

/**
 * @ingroup hapticFeedback
 * @brief Receives FT sensor data and commands a Cartesian controller.
 */
class HapticFeedback : public yarp::os::RFModule
{
public:
    ~HapticFeedback() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;
    double getPeriod() override;

private:
    yarp::dev::PolyDriver sensorClientDevice;
    yarp::dev::ISixAxisForceTorqueSensors * iSensor {nullptr};

    yarp::dev::PolyDriver cartesianClientDevice;
    ICartesianControl * iCartesianControl {nullptr};

    double period {0.0};
    double scaling {0.0};
    double threshold {0.0};

    KDL::Frame H_device_sensor;
};

} // namespace roboticslab

#endif // __HAPTIC_FEEDBACK_HPP__

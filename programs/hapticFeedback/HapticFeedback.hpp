#ifndef __HAPTIC_FEEDBACK_HPP__
#define __HAPTIC_FEEDBACK_HPP__

#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>

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
};

} // namespace roboticslab

#endif // __HAPTIC_FEEDBACK_HPP__

#ifndef __XTRAINER_DEVICE_HPP__
#define __XTRAINER_DEVICE_HPP__

#include "StreamingDevice.hpp"

#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include "ICartesianSolver.h"

namespace roboticslab
{

/**
 * @ingroup streamingDeviceController
 *
 * @brief Represents an XTrainer-compatible device.
 */
class XTrainerDevice : public StreamingDevice
{
public:
    XTrainerDevice(yarp::os::Searchable & config);

    bool acquireInterfaces() override;
    bool initialize(const ICartesianControl::config_map_t & params) override;
    bool acquireData() override;
    bool transformData(double scaling) override;
    ICartesianControl::Actuator getActuatorState() override;
    void sendMovementCommand(double timestamp) override;

private:
    yarp::dev::IJoypadController * iJoypadController {nullptr};
    ICartesianSolver * iCartesianSolver {nullptr};
    yarp::dev::PolyDriver solverDevice;

    std::string kinematics;

    std::vector<double> initialTcpOffset;
    std::vector<double> initialDeviceOffset;

    KDL::Frame H_base_device, H_ee_device, H_device_ee;

    bool buttonClose {false};
    bool buttonOpen {false};
};

} // namespace roboticslab

#endif // __XTRAINER_DEVICE_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __VISUAL_SERVO_CONTROLLER_HPP__
#define __VISUAL_SERVO_CONTROLLER_HPP__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"
#if 0
# include "IProximitySensors.h"
#endif
#include "GrabberResponder.hpp"

namespace roboticslab
{

/**
 * @ingroup visualServoController
 *
 * @brief Create seek-and-follow trajectories based on
 * visual servoing algorithms.
 */
class VisualServoController : public yarp::os::RFModule
{
public:
    ~VisualServoController() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;
    double getPeriod() override;

private:
    GrabberResponder grabberResponder;
    yarp::os::BufferedPort<yarp::os::Bottle> grabberPort;

    yarp::dev::PolyDriver cartesianControlDevice;
    roboticslab::ICartesianControl * iCartesianControl {nullptr};

#if 0
    yarp::dev::PolyDriver sensorsClientDevice;
    roboticslab::IProximitySensors * iProximitySensors {nullptr};
#endif

    double period;
};

} // namespace roboticslab

#endif // __VISUAL_SERVO_CONTROLLER_HPP__

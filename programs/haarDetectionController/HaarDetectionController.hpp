// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTION_CONTROLLER_HPP__
#define __HAAR_DETECTION_CONTROLLER_HPP__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"
#include "IProximitySensors.h"

#include "GrabberResponder.hpp"

namespace roboticslab
{

/**
 * @ingroup haarDetectionController
 *
 * @brief Create seek-and-follow trajectories based on
 * Haar detection algorithms.
 */
class HaarDetectionController : public yarp::os::RFModule
{
public:
    ~HaarDetectionController()
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
    roboticslab::ICartesianControl * iCartesianControl;

    yarp::dev::PolyDriver sensorsClientDevice;
    roboticslab::IProximitySensors * iProximitySensors;

    double period;
};

} // namespace roboticslab

#endif // __HAAR_DETECTION_CONTROLLER_HPP__

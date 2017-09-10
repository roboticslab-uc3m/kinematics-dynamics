// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTION_CONTROLLER_HPP__
#define __HAAR_DETECTION_CONTROLLER_HPP__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"

#include "GrabberResponder.hpp"

#define DEFAULT_LOCAL_PORT "/HaarDetectionControl"
#define DEFAULT_REMOTE_VISION "/haarDetection2D"
#define DEFAULT_REMOTE_CARTESIAN "/CartesianControl"

#define DEFAULT_PERIOD 0.01 // [s]

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

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();

private:

    GrabberResponder grabberResponder;
    yarp::os::BufferedPort<yarp::os::Bottle> grabberPort;

    yarp::dev::PolyDriver cartesianControlDevice;
    roboticslab::ICartesianControl *iCartesianControl;

    double period;
};

}  // namespace roboticslab

#endif  // __HAAR_DETECTION_CONTROLLER_HPP__

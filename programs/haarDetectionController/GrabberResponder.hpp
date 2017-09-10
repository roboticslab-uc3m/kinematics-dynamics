// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __GRABBER_RESPONDER_HPP__
#define __GRABBER_RESPONDER_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/PortReaderBuffer.h>

#include "ICartesianControl.h"

namespace roboticslab
{

/**
 * @ingroup haarDetectionController
 *
 * @brief Callback class for dealing with incoming grabber
 * data streams.
 */
class GrabberResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:

    GrabberResponder() : iCartesianControl(NULL),
                         isStopped(true)
    {}

    virtual void onRead(yarp::os::Bottle &b);

    void setICartesianControlDriver(roboticslab::ICartesianControl *iCartesianControl)
    {
        this->iCartesianControl = iCartesianControl;
    }

private:

    roboticslab::ICartesianControl *iCartesianControl;

    bool isStopped;
};

}  // namespace roboticslab

#endif  // __GRABBER_RESPONDER_HPP__

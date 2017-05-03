// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PREMULT_PORTS__
#define __PREMULT_PORTS__

#include <stdlib.h>

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>


namespace teo
{

/**
 * @ingroup cv1ToRoot
 *
 * @brief Implements a port with x callbacks.
 */
class PremultPorts : public yarp::os::BufferedPort<yarp::os::Bottle> {
protected:
    /**
    * Implement the actual callback.
    */
    void onRead(yarp::os::Bottle& b);
    yarp::os::Port* outPort;

public:

    PremultPorts() {}
    void setOutPort(yarp::os::Port* _outPort);
};

}  // namespace teo

#endif


// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_GRAVITY_COMPENSATOR__
#define __TEO_GRAVITY_COMPENSATOR__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "ColorDebug.hpp"

#define DEFAULT_SOLVER "kdlsolver"

using namespace yarp::os;
using namespace yarp::dev;

/**
 * @ingroup TeoGravityCompensator
 *
 * The TeoGravityCompensator class implements a server part that receives a connection from a remote
 * \ref cartesianServer module.
 * 
 */
class TeoGravityCompensator : public RFModule {
protected:
    yarp::dev::PolyDriver robotDevice;

    yarp::dev::IPositionControl *ipos;

    bool updateModule();
    bool interruptModule();
    // double getPeriod();

public:
    TeoGravityCompensator();
    bool configure(ResourceFinder &rf);
};

#endif  // __TEO_GRAVITY_COMPENSATOR__


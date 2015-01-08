// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_SERVER__
#define __CARTESIAN_SERVER__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "TeoXRpcResponder.hpp"
#include "TeoXCallbackPort.hpp"

#define DEFAULT_CONTROLLER "kdlcontroller"
#define DEFAULT_PREFIX "/teoSim/rightArm"
#define DEFAULT_MOVJ_LOCAL "/teoSim/rightArm/movjCartesianServer"
#define DEFAULT_MOVJ_REMOTE "/teoSim/rightArm"

using namespace yarp::os;
using namespace yarp::dev;

/**
 * @ingroup asibot_libraries
 *
 * \defgroup CartesianServerLib
 *
 * The \ref CartesianServerLib library is composed by a single class, CartesianServer.
 *
 */

/**
 * @ingroup CartesianServer
 *
 * The CartesianServer class implements a server part that receives a connection from a remote
 * \ref cartesianServer module.
 * 
 */
class TeoCartesianServer : public RFModule {
protected:
    yarp::dev::PolyDriver cartesianDevice;
    yarp::dev::PolyDriver robotDevice;

    yarp::dev::ICartesianControl *icart;
    yarp::dev::IPositionControl *ipos;
    int *csStatus;

    RpcServer xRpcServer;
	TeoXRpcResponder xResponder;

    TeoXCallbackPort xPort;

    bool updateModule();
    bool interruptModule();
    // double getPeriod();

public:
    TeoCartesianServer();
    bool configure(ResourceFinder &rf);
};

#endif


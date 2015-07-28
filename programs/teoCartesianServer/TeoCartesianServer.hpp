// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_CARTESIAN_SERVER__
#define __TEO_CARTESIAN_SERVER__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "ColorDebug.hpp"
#include "TeoXRpcResponder.hpp"
#include "TeoXCallbackPort.hpp"
#include "CartesianRateThread.hpp"

#define DEFAULT_SOLVER "KdlSolver"
#define DEFAULT_KINEMATICS "rightArmKinematics.ini"
#define DEFAULT_REMOTE "/teoSim/rightArm"

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 * @ingroup teoCartesianServer
 *
 * @brief Implements a server part that receives a connection from a remote
 * \ref cartesianServer module.
 * 
 */
class TeoCartesianServer : public RFModule {
protected:
    yarp::dev::PolyDriver solverDevice;
    yarp::dev::PolyDriver robotDevice;

    RpcServer xRpcServer;
	TeoXRpcResponder xResponder;

    TeoXCallbackPort xPort;

    bool updateModule();
    bool interruptModule();
    // double getPeriod();

    CartesianRateThread cartesianRateThread;

public:
    TeoCartesianServer();
    bool configure(ResourceFinder &rf);
};

}  // namespace teo

#endif  // __TEO_CARTESIAN_SERVER__


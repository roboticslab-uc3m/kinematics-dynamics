#ifndef __PREMULT_PORTS__
#define __PREMULT_PORTS__

#include <stdlib.h>

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/api.h>
#include <yarp/dev/IEncoders.h>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "ICartesianSolver.h"

#include "KdlVectorConverter.hpp"

#include "ColorDebug.hpp"

namespace teo
{

/**
 * @ingroup transCoordsUsingJoints
 *
 * @brief Implements a port with x callbacks.
 */
class PremultPorts : public yarp::os::BufferedPort<yarp::os::Bottle>, KdlVectorConverter
{

public:

    PremultPorts();

    void setOutPort(yarp::os::Port* outPort);
    void setIEncoders(yarp::dev::IEncoders* iEncoders);
    void setICartesianSolver(teo::ICartesianSolver* iCartesianSolver);

private:

    /** Implement the actual callback. */
    void onRead(yarp::os::Bottle& b);

    yarp::os::Port* outPort;

    yarp::dev::IEncoders* iEncoders;
    int numRobotJoints;

    teo::ICartesianSolver* iCartesianSolver;

};

}  // namespace teo

#endif


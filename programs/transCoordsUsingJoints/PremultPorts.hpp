#ifndef __PREMULT_PORTS__
#define __PREMULT_PORTS__

#include <stdlib.h>

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include <yarp/dev/IEncoders.h>

#include "ICartesianSolver.h"

namespace roboticslab
{

/**
 * @ingroup transCoordsUsingJoints
 *
 * @brief Implements a port with x callbacks.
 */
class PremultPorts : public yarp::os::BufferedPort<yarp::os::Bottle>
{

public:

    PremultPorts() {}

    void setOutPort(yarp::os::Port* outPort);
    void setIEncoders(yarp::dev::IEncoders* iEncoders);
    void setICartesianSolver(roboticslab::ICartesianSolver* iCartesianSolver);

private:

    /** Implement the actual callback. */
    void onRead(yarp::os::Bottle& b);

    yarp::os::Port* outPort;

    yarp::dev::IEncoders* iEncoders;
    int numRobotJoints;

    roboticslab::ICartesianSolver* iCartesianSolver;

};

}  // namespace roboticslab

#endif


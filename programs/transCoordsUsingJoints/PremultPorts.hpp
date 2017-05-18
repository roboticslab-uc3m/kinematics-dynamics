#ifndef __PREMULT_PORTS__
#define __PREMULT_PORTS__

#include <stdlib.h>

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/api.h>

// Temporary fix to avoid https://github.com/roboticslab-uc3m/kinematics-dynamics/issues/97
// Affects YARP 2.3.68 and 2.3.69-2.3.69.6
#include <yarp/conf/version.h>
#define __YARP_VERSION_N (YARP_VERSION_MAJOR * 100000 \
                          + YARP_VERSION_MINOR * 10000 \
                          + YARP_VERSION_PATCH * 10)
#if (YARP_VERSION_TWEAK + 0)  // may be empty
#define __YARP_VERSION_N (__YARP_VERSION_N + YARP_VERSION_TWEAK)
#endif
#if __YARP_VERSION_N == 230680 || (__YARP_VERSION_N >= 230690 && __YARP_VERSION_N <= 230696)
#include <yarp/dev/IEncoders.h>
#endif
#undef __YARP_VERSION_N

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


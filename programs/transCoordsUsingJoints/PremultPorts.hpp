#ifndef __PREMULT_PORTS__
#define __PREMULT_PORTS__

#include <stdlib.h>

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/IEncoders.h>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>


namespace teo
{

/**
 * @ingroup transCoordsUsingJoints
 *
 * @brief Implements a port with x callbacks.
 */
class PremultPorts : public yarp::os::BufferedPort<yarp::os::Bottle>
{

public:

    void setOutPort(yarp::os::Port* outPort);

private:

    /** Implement the actual callback. */
    void onRead(yarp::os::Bottle& b);

    yarp::os::Port* outPort;

    KDL::Chain oneChain;

};

}  // namespace teo

#endif


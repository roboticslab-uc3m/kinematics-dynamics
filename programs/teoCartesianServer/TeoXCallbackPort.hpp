// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __X_CALLBACK_PORT__
#define __X_CALLBACK_PORT__

#define _USE_MATH_DEFINES  // this is only needed in Windows

#include <math.h>

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 * @ingroup teoCartesianServer
 *
 * @brief Implements a port with x callbacks.
 */
class TeoXCallbackPort : public BufferedPort<Bottle> {
    protected:
        /** Implement the actual callback. */
        void onRead(Bottle& b);

    public:

        TeoXCallbackPort() {}

};

}  // namespace teo

#endif


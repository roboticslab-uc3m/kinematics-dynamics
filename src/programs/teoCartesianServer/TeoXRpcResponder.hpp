// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __X_RPC_RESPONDER__
#define __X_RPC_RESPONDER__

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#define MAX_NUM_MOTORS 100

#define VOCAB_HELP VOCAB4('h','e','l','p')
#define VOCAB_MY_STOP VOCAB4('s','t','o','p')
#define VOCAB_STAT VOCAB4('s','t','a','t')
#define VOCAB_MOVL VOCAB4('m','o','v','l')
#define VOCAB_MOVJ VOCAB4('m','o','v','j')
#define VOCAB_INV VOCAB3('i','n','v')
#define VOCAB_MOVE VOCAB4('m','o','v','e')
#define VOCAB_WAIT VOCAB4('w','a','i','t')
#define VOCAB_TOOL VOCAB4('t','o','o','l')

using namespace yarp::os;
using namespace yarp::dev;
using yarp::sig::Vector;

namespace teo
{

/**
 * @ingroup xRpcResponder
 *
 * TeoXRpcResponder class implements an xRpcPort responder (callback on RPC).
 */
class TeoXRpcResponder : public PortReader {

    protected:
        /**
        * Implement the actual responder (callback on RPC).
        */
        virtual bool read(ConnectionReader& connection);

        yarp::dev::ICartesianControl *icart;
        yarp::dev::IPositionControl *ipos;
        int *csStatus;

    public:

        /** Register a cartesian interface for the PortReader. */
        void setCartesianInterface(yarp::dev::ICartesianControl* _icart);

        /** Register a position interface for the PortReader. */
        void setPositionInterface(yarp::dev::IPositionControl* _ipos);

        /** Register. */
        void setCsStatus(int* _csStatus);

};

}  // namespace teo

#endif


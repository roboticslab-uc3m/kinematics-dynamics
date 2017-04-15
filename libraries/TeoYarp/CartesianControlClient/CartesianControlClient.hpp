// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_CLIENT_HPP__
#define __CARTESIAN_CONTROL_CLIENT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream> // only windows

#include "ICartesianControl.h"

#include "ColorDebug.hpp"

#define VOCAB_FAILED VOCAB4('f','a','i','l')  //-- from yarp::dev

#define DEFAULT_CARTESIAN_LOCAL "/CartesianControl"
#define DEFAULT_CARTESIAN_REMOTE "/CartesianControl"

namespace teo
{

/**
 * @ingroup TeoYarp
 * \defgroup CartesianControlClient
 *
 * @brief Contains teo::CartesianControlClient.
 */

/**
 * @ingroup CartesianControlClient
 * @brief The CartesianControlClient class implements ICartesianControl client side.
 */

class CartesianControlClient : public yarp::dev::DeviceDriver, public ICartesianControl {

    public:

        CartesianControlClient() {}

        // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp--
        /** Inform on control state, and get robot position and perform forward kinematics. */
        virtual bool stat(int &state, std::vector<double> &x);

        /** Perform inverse kinematics (using robot position as initial guess) but do not move. */
        virtual bool inv(const std::vector<double> &xd, std::vector<double> &q);

        /** movj */
        virtual bool movj(const std::vector<double> &xd);

        /** relj */
        virtual bool relj(const std::vector<double> &xd);

        /** movl */
        virtual bool movl(const std::vector<double> &xd);

        /** movv */
        virtual bool movv(const std::vector<double> &xdotd);

        /** gcmp */
        virtual bool gcmp();

        /** forc */
        virtual bool forc(const std::vector<double> &td);

        /** stop */
        virtual bool stopControl();

        // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

        /**
        * Open the DeviceDriver.
        * @param config is a list of parameters for the device.
        * Which parameters are effective for your device can vary.
        * See \ref dev_examples "device invocation examples".
        * If there is no example for your device,
        * you can run the "yarpdev" program with the verbose flag
        * set to probe what parameters the device is checking.
        * If that fails too,
        * you'll need to read the source code (please nag one of the
        * yarp developers to add documentation for your device).
        * @return true/false upon success/failure
        */
        virtual bool open(yarp::os::Searchable& config);

        /**
        * Close the DeviceDriver.
        * @return true/false on success/failure.
        */
        virtual bool close();

protected:

    yarp::os::RpcClient rpcClient;

};

}  // namespace teo

#endif  // __CARTESIAN_CONTROL_CLIENT_HPP__


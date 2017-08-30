// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_CLIENT_HPP__
#define __CARTESIAN_CONTROL_CLIENT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <vector>
#include <iostream> // only windows

#include "ICartesianControl.h"

#define VOCAB_FAILED VOCAB4('f','a','i','l')  //-- from yarp::dev

#define DEFAULT_CARTESIAN_LOCAL "/CartesianControl"
#define DEFAULT_CARTESIAN_REMOTE "/CartesianControl"

namespace roboticslab
{

/**
 * @ingroup TeoYarp
 * \defgroup CartesianControlClient
 *
 * @brief Contains roboticslab::CartesianControlClient.
 */

/**
 * @ingroup CartesianControlClient
 * @brief The CartesianControlClient class implements ICartesianControl client side.
 */

class CartesianControlClient : public yarp::dev::DeviceDriver, public ICartesianControl
{

    public:

        CartesianControlClient() {}

        // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp--

        virtual bool stat(int &state, std::vector<double> &x);

        virtual bool inv(const std::vector<double> &xd, std::vector<double> &q);

        virtual bool movj(const std::vector<double> &xd);

        virtual bool relj(const std::vector<double> &xd);

        virtual bool movl(const std::vector<double> &xd);

        virtual bool movv(const std::vector<double> &xdotd);

        virtual bool gcmp();

        virtual bool forc(const std::vector<double> &td);

        virtual bool stopControl();

        virtual bool tool(const std::vector<double> &x);

        /** fwd */
        virtual bool fwd(const std::vector<double> &rot, double step);

        /** bkwd*/
        virtual bool bkwd(const std::vector<double> &rot, double step);

        /** rot */
        virtual bool rot(const std::vector<double> &rot);

        /** vmos */
        virtual bool vmos(const std::vector<double> &xdot);

        /** pose */
        virtual bool pose(const std::vector<double> &x, double interval);

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

    yarp::os::Port commandPort;

    yarp::os::PortWriterBuffer<yarp::os::Bottle> commandBuffer;

};

}  // namespace roboticslab

#endif  // __CARTESIAN_CONTROL_CLIENT_HPP__

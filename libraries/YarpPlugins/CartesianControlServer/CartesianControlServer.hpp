// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_HPP__
#define __CARTESIAN_CONTROL_SERVER_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RpcServer.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <vector>

#include "ICartesianControl.h"
#include "KinematicRepresentation.hpp"

#define DEFAULT_PREFIX "/CartesianServer"
#define DEFAULT_MS 20

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup CartesianControlServer
 *
 * @brief Contains roboticslab::CartesianControlServer.
 */

class RpcResponder;
class RpcTransformResponder;
class StreamResponder;

/**
 * @ingroup CartesianControlServer
 * @brief The CartesianControlServer class implements ICartesianControl server side.
 */

class CartesianControlServer : public yarp::dev::DeviceDriver, public yarp::os::PeriodicThread
{
public:

    CartesianControlServer()
        : yarp::os::PeriodicThread(DEFAULT_MS * 0.001),
          iCartesianControl(NULL),
          rpcResponder(NULL), rpcTransformResponder(NULL),
          streamResponder(NULL),
          fkStreamEnabled(true)
    {}

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

    /**
     * Loop function. This is the thread itself.
     */
    virtual void run();

protected:

    yarp::dev::PolyDriver cartesianControlDevice;

    yarp::os::RpcServer rpcServer, rpcTransformServer;
    yarp::os::BufferedPort<yarp::os::Bottle> fkOutPort, commandPort;

    roboticslab::ICartesianControl *iCartesianControl;

    RpcResponder *rpcResponder, *rpcTransformResponder;
    StreamResponder *streamResponder;

    bool fkStreamEnabled;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to RPC command messages.
 */
class RpcResponder : public yarp::dev::DeviceResponder
{
public:

    RpcResponder(roboticslab::ICartesianControl *iCartesianControl)
    {
        this->iCartesianControl = iCartesianControl;

        // shadows DeviceResponder::makeUsage(), which was already called by the base constructor
        makeUsage();
    }

    /**
     * Respond to a message.
     * @param in the message
     * @param out the response
     * @return true if there was no critical failure
     */
    virtual bool respond(const yarp::os::Bottle& in, yarp::os::Bottle& out);

    /**
     * Generate command usage information.
     */
    void makeUsage();

protected:

    typedef bool (ICartesianControl::*RunnableFun)();
    typedef bool (ICartesianControl::*ConsumerFun)(const std::vector<double>&);
    typedef bool (ICartesianControl::*FunctionFun)(const std::vector<double>&, std::vector<double>&);

    bool handleStatMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out);
    bool handleWaitMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out);

    bool handleRunnableCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, RunnableFun cmd);
    bool handleConsumerCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, ConsumerFun cmd);
    bool handleFunctionCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, FunctionFun cmd);

    bool handleParameterSetter(const yarp::os::Bottle& in, yarp::os::Bottle& out);
    bool handleParameterGetter(const yarp::os::Bottle& in, yarp::os::Bottle& out);

    bool handleParameterSetterGroup(const yarp::os::Bottle& in, yarp::os::Bottle& out);
    bool handleParameterGetterGroup(const yarp::os::Bottle& in, yarp::os::Bottle& out);

    virtual bool transformIncomingData(std::vector<double>& vin)
    {
        return true;
    }

    virtual bool transformOutgoingData(std::vector<double>& vout)
    {
        return true;
    }

    roboticslab::ICartesianControl *iCartesianControl;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to RPC command messages, transforms incoming data.
 */
class RpcTransformResponder : public RpcResponder
{
public:

    RpcTransformResponder(roboticslab::ICartesianControl * iCartesianControl, KinRepresentation::orientation_system orient)
        : RpcResponder(iCartesianControl),
          orient(orient)
    {}

protected:

    virtual bool transformIncomingData(std::vector<double>& vin);
    virtual bool transformOutgoingData(std::vector<double>& vout);

    KinRepresentation::orientation_system orient;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to streaming command messages.
 */
class StreamResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:

    StreamResponder(roboticslab::ICartesianControl *iCartesianControl)
    {
        this->iCartesianControl = iCartesianControl;
    }

    void onRead(yarp::os::Bottle& b);

protected:

    typedef void (ICartesianControl::*ConsumerFun)(const std::vector<double>&);
    typedef void (ICartesianControl::*BiConsumerFun)(const std::vector<double>&, double);

    void handleConsumerCmdMsg(const yarp::os::Bottle& in, ConsumerFun cmd);
    void handleBiConsumerCmdMsg(const yarp::os::Bottle& in, BiConsumerFun cmd);

    roboticslab::ICartesianControl *iCartesianControl;
};

}  // namespace roboticslab

#endif  // __CARTESIAN_CONTROL_SERVER_HPP__

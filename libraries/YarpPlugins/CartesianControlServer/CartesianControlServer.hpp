// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_HPP__
#define __CARTESIAN_CONTROL_SERVER_HPP__

#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RpcServer.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"
#include "KinematicRepresentation.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CartesianControlServer
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
class CartesianControlServer : public yarp::dev::DeviceDriver,
                               public yarp::os::PeriodicThread
{
public:
    CartesianControlServer() : yarp::os::PeriodicThread(1.0)
    {}

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------
    void run() override;

protected:
    yarp::dev::PolyDriver cartesianControlDevice;

    yarp::os::RpcServer rpcServer, rpcTransformServer;
    yarp::os::BufferedPort<yarp::os::Bottle> fkOutPort, commandPort;

    roboticslab::ICartesianControl * iCartesianControl {nullptr};

    RpcResponder * rpcResponder {nullptr};
    RpcResponder * rpcTransformResponder {nullptr};
    StreamResponder * streamResponder {nullptr};

    bool fkStreamEnabled;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to RPC command messages.
 */
class RpcResponder : public yarp::dev::DeviceResponder
{
public:
    RpcResponder(roboticslab::ICartesianControl * _iCartesianControl)
        : iCartesianControl(_iCartesianControl)
    {
        // shadows DeviceResponder::makeUsage(), which was already called by the base constructor
        makeUsage();
    }

    /**
     * Respond to a message.
     * @param in the message
     * @param out the response
     * @return true if there was no critical failure
     */
    bool respond(const yarp::os::Bottle & in, yarp::os::Bottle & out) override;

    /**
     * Generate command usage information.
     */
    void makeUsage();

protected:
    virtual bool transformIncomingData(std::vector<double> & vin)
    { return true; }

    virtual bool transformOutgoingData(std::vector<double> & vout)
    { return true; }

private:
    using RunnableFun = bool (ICartesianControl::*)();
    using ConsumerFun = bool (ICartesianControl::*)(const std::vector<double> &);
    using FunctionFun = bool (ICartesianControl::*)(const std::vector<double> &, std::vector<double> &);

    bool handleStatMsg(const yarp::os::Bottle & in, yarp::os::Bottle & out);
    bool handleWaitMsg(const yarp::os::Bottle & in, yarp::os::Bottle & out);
    bool handleActMsg(const yarp::os::Bottle & in, yarp::os::Bottle & out);

    bool handleRunnableCmdMsg(const yarp::os::Bottle & in, yarp::os::Bottle & out, RunnableFun cmd);
    bool handleConsumerCmdMsg(const yarp::os::Bottle & in, yarp::os::Bottle & out, ConsumerFun cmd);
    bool handleFunctionCmdMsg(const yarp::os::Bottle & in, yarp::os::Bottle & out, FunctionFun cmd);

    bool handleParameterSetter(const yarp::os::Bottle & in, yarp::os::Bottle & out);
    bool handleParameterGetter(const yarp::os::Bottle & in, yarp::os::Bottle & out);

    bool handleParameterSetterGroup(const yarp::os::Bottle & in, yarp::os::Bottle & out);
    bool handleParameterGetterGroup(const yarp::os::Bottle & in, yarp::os::Bottle & out);

    roboticslab::ICartesianControl * iCartesianControl;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to RPC command messages, transforms incoming data.
 */
class RpcTransformResponder : public RpcResponder
{
public:
    RpcTransformResponder(roboticslab::ICartesianControl * iCartesianControl,
                          KinRepresentation::coordinate_system coord,
                          KinRepresentation::orientation_system orient,
                          KinRepresentation::angular_units units)
        : RpcResponder(iCartesianControl),
          coord(coord),
          orient(orient),
          units(units)
    {}

private:
    bool transformIncomingData(std::vector<double> & vin) override;
    bool transformOutgoingData(std::vector<double> & vout) override;

    KinRepresentation::coordinate_system coord;
    KinRepresentation::orientation_system orient;
    KinRepresentation::angular_units units;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to streaming command messages.
 */
class StreamResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    StreamResponder(roboticslab::ICartesianControl * _iCartesianControl)
        : iCartesianControl(_iCartesianControl)
    {}

    void onRead(yarp::os::Bottle & b) override;

private:
    using ConsumerFun = void (ICartesianControl::*)(const std::vector<double> &);
    using BiConsumerFun = void (ICartesianControl::*)(const std::vector<double> &, double);

    void handleConsumerCmdMsg(const yarp::os::Bottle & in, ConsumerFun cmd);
    void handleBiConsumerCmdMsg(const yarp::os::Bottle & in, BiConsumerFun cmd);

    roboticslab::ICartesianControl * iCartesianControl;
};

} // namespace roboticslab

#endif // __CARTESIAN_CONTROL_SERVER_HPP__

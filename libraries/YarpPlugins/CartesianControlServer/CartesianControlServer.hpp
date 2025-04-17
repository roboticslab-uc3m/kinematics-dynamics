// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_HPP__
#define __CARTESIAN_CONTROL_SERVER_HPP__

#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RpcServer.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/WrapperSingle.h>

#include "ICartesianControl.h"
#include "KinematicRepresentation.hpp"
#include "CartesianControlServer_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CartesianControlServer
 *
 * @brief Contains CartesianControlServer.
 */

namespace roboticslab
{
    class RpcResponder;
    class RpcTransformResponder;
    class StreamResponder;
}

/**
 * @ingroup CartesianControlServer
 * @brief The CartesianControlServer class implements ICartesianControl server side.
 */
class CartesianControlServer : public yarp::dev::DeviceDriver,
                               public yarp::dev::WrapperSingle,
                               public yarp::os::PeriodicThread,
                               public CartesianControlServer_ParamsParser
{
public:
    CartesianControlServer() : yarp::os::PeriodicThread(1.0)
    {}

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // -------- IWrapper declarations. Implementation in IWrapperImpl.cpp --------
    bool attach(yarp::dev::PolyDriver * poly) override;
    bool detach() override;

    // -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------
    void run() override;

protected:
    yarp::os::RpcServer rpcServer, rpcTransformServer;
    yarp::os::BufferedPort<yarp::os::Bottle> fkOutPort, commandPort;

    roboticslab::ICartesianControl * iCartesianControl {nullptr};

    roboticslab::RpcResponder * rpcResponder {nullptr};
    roboticslab::RpcResponder * rpcTransformResponder {nullptr};
    roboticslab::StreamResponder * streamResponder {nullptr};
};

namespace roboticslab
{

/**
 * @ingroup CartesianControlServer
 * @brief Responds to RPC command messages.
 */
class RpcResponder : public yarp::dev::DeviceResponder
{
public:
    RpcResponder()
    {
        // shadows DeviceResponder::makeUsage(), which was already called by the base constructor
        makeUsage();
    }

    void setHandle(ICartesianControl * _iCartesianControl)
    { iCartesianControl = _iCartesianControl; }

    bool respond(const yarp::os::Bottle & in, yarp::os::Bottle & out) override;
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

    ICartesianControl * iCartesianControl;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to RPC command messages, transforms incoming data.
 */
class RpcTransformResponder : public RpcResponder
{
public:
    RpcTransformResponder(KinRepresentation::coordinate_system coord,
                          KinRepresentation::orientation_system orient,
                          KinRepresentation::angular_units units)
        : coord(coord),
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
    void setHandle(ICartesianControl * _iCartesianControl)
    { iCartesianControl = _iCartesianControl;}

    void onRead(yarp::os::Bottle & b) override;

private:
    using ConsumerFun = void (ICartesianControl::*)(const std::vector<double> &);
    using BiConsumerFun = void (ICartesianControl::*)(const std::vector<double> &, double);

    void handleConsumerCmdMsg(const yarp::os::Bottle & in, ConsumerFun cmd);
    void handleBiConsumerCmdMsg(const yarp::os::Bottle & in, BiConsumerFun cmd);

    ICartesianControl * iCartesianControl {nullptr};
};

} // namespace roboticslab

#endif // __CARTESIAN_CONTROL_SERVER_HPP__

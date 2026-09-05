// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_HPP__
#define __CARTESIAN_CONTROL_SERVER_HPP__

#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/TypedReaderCallback.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/WrapperSingle.h>

#include "ICartesianControl.h"
#include "CartesianControlMsgs.h"
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

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
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
 * @brief Base class for all responders.
 */
class ResponderBase
{
public:
    virtual ~ResponderBase() = 0;

    void setHandle(ICartesianControl * _iCartesianControl)
    { iCartesianControl = _iCartesianControl; }

    void unsetHandle()
    { iCartesianControl = nullptr; }

protected:
    ICartesianControl * iCartesianControl {nullptr};
};

inline ResponderBase::~ResponderBase() {}

/**
 * @ingroup CartesianControlServer
 * @brief Responds to RPC command messages.
 */
class RpcResponder : public CartesianControlMsgs,
                     public ResponderBase
{
public:
    return_get_state getState() override;
    return_solve_pose solvePose(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveJoint(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveLinear(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveVelocity(const std::vector<double> & xdotd) override;
    yarp::dev::ReturnValue gravityCompensation() override;
    yarp::dev::ReturnValue forceControl(const std::vector<double> & fd) override;
    yarp::dev::ReturnValue stopControl() override;
    yarp::dev::ReturnValue changeTool(const std::vector<double> & x) override;
    yarp::dev::ReturnValue actuateTool(roboticslab::ICartesianControl::Actuator command) override;
    yarp::dev::ReturnValue setParameterDouble(roboticslab::ICartesianControl::Config vocab, double value) override;
    yarp::dev::ReturnValue setParameterVocab(roboticslab::ICartesianControl::Config vocab, std::int32_t value) override;
    return_get_parameter_double getParameterDouble(roboticslab::ICartesianControl::Config vocab) override;
    return_get_parameter_vocab getParameterVocab(roboticslab::ICartesianControl::Config vocab) override;
    yarp::dev::ReturnValue setParameters(const std::map<roboticslab::ICartesianControl::Config, double> & params) override;
    return_get_parameters getParameters() override;

private:
    using ResponderBase::iCartesianControl;
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

    return_get_state getState() override;
    return_solve_pose solvePose(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveJoint(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveLinear(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue changeTool(const std::vector<double> & x) override;

private:
    bool transformIncomingData(const std::vector<double> & vin, std::vector<double> & vout);
    bool transformOutgoingData(const std::vector<double> & vin, std::vector<double> & vout);

    KinRepresentation::coordinate_system coord;
    KinRepresentation::orientation_system orient;
    KinRepresentation::angular_units units;
};

/**
 * @ingroup CartesianControlServer
 * @brief Responds to streaming command messages.
 */
class StreamResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>,
                        public ResponderBase
{
public:
    void onRead(yarp::os::Bottle & b) override;

private:
    using ConsumerFun = void (ICartesianControl::*)(const std::vector<double> &);
    void handleConsumerCmdMsg(const yarp::os::Bottle & in, ConsumerFun cmd);
};

} // namespace roboticslab

#endif // __CARTESIAN_CONTROL_SERVER_HPP__

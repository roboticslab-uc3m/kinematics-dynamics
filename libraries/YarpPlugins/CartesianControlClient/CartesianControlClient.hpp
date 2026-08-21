// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_CLIENT_HPP__
#define __CARTESIAN_CONTROL_CLIENT_HPP__

#include <mutex>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/RpcClient.h>

#include <yarp/dev/Drivers.h>

#include "ICartesianControl.h"
#include "CartesianControlMsgs.h"
#include "CartesianControlClient_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CartesianControlClient
 *
 * @brief Contains CartesianControlClient.
 */

 namespace roboticslab
 {

/**
 * @ingroup CartesianControlClient
 * @brief Responds to streaming FK messages.
 */
class FkStreamResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    FkStreamResponder();
    void onRead(yarp::os::Bottle& b) override;
    bool getLastStatData(std::vector<double> &x, roboticslab::ICartesianControl::State * state, double * timestamp, double timeout);

private:
    double localArrivalTime;
    int state;
    double timestamp;
    std::vector<double> x;
    mutable std::mutex mtx;
};

} // namespace roboticslab

/**
 * @ingroup CartesianControlClient
 * @brief The CartesianControlClient class implements ICartesianControl client side.
 */
class CartesianControlClient : public yarp::dev::DeviceDriver,
                               public roboticslab::ICartesianControl,
                               public CartesianControlClient_ParamsParser
{
public:
    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --

    // RPC commands
    yarp::dev::ReturnValue stat(std::vector<double> & x, roboticslab::ICartesianControl::State * state = nullptr, double * timestamp = nullptr) override;
    yarp::dev::ReturnValue inv(const std::vector<double> & xd, std::vector<double> & q) override;
    yarp::dev::ReturnValue movj(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue relj(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue movl(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue movv(const std::vector<double> & xdotd) override;
    yarp::dev::ReturnValue gcmp() override;
    yarp::dev::ReturnValue forc(const std::vector<double> & fd) override;
    yarp::dev::ReturnValue stopControl() override;
    yarp::dev::ReturnValue wait(double timeout) override;
    yarp::dev::ReturnValue tool(const std::vector<double> & x) override;
    yarp::dev::ReturnValue act(roboticslab::ICartesianControl::Actuator command) override;

    // streaming commands
    void pose(const std::vector<double> & x) override;
    void twist(const std::vector<double> & xdot) override;
    void wrench(const std::vector<double> & w) override;

    // configuration getters/setters
    yarp::dev::ReturnValue setParameter(roboticslab::ICartesianControl::Config vocab, double value) override;
    yarp::dev::ReturnValue getParameter(roboticslab::ICartesianControl::Config vocab, double * value) override;
    yarp::dev::ReturnValue setParameters(const std::map<roboticslab::ICartesianControl::Config, double> & params) override;
    yarp::dev::ReturnValue getParameters(std::map<roboticslab::ICartesianControl::Config, double> & params) override;

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    void handleStreamingConsumerCmd(roboticslab::ICartesianControl::Streaming vocab, const std::vector<double> & in);

    yarp::os::RpcClient rpcClient;
    yarp::os::BufferedPort<yarp::os::Bottle> fkInPort;
    yarp::os::BufferedPort<yarp::os::Bottle> commandPort;

    roboticslab::CartesianControlMsgs rpcSender;
    roboticslab::FkStreamResponder fkStreamResponder;
};

#endif // __CARTESIAN_CONTROL_CLIENT_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_CLIENT_HPP__
#define __CARTESIAN_CONTROL_CLIENT_HPP__

#include <mutex>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/RpcClient.h>

#include <yarp/dev/Drivers.h>

#include <vector>

#include "ICartesianControl.h"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup CartesianControlClient
 *
 * @brief Contains roboticslab::CartesianControlClient.
 */

/**
 * @ingroup CartesianControlClient
 * @brief Responds to streaming FK messages.
 */
class FkStreamResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    FkStreamResponder();
    void onRead(yarp::os::Bottle& b) override;
    bool getLastStatData(std::vector<double> &x, int *state, double * timestamp, double timeout);

private:
    double localArrivalTime;
    int state;
    double timestamp;
    std::vector<double> x;
    mutable std::mutex mtx;
};

/**
 * @ingroup CartesianControlClient
 * @brief The CartesianControlClient class implements ICartesianControl client side.
 */
class CartesianControlClient : public yarp::dev::DeviceDriver,
                               public ICartesianControl
{
public:
    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp--
    bool stat(std::vector<double> &x, int * state = nullptr, double * timestamp = nullptr) override;
    bool inv(const std::vector<double> &xd, std::vector<double> &q) override;
    bool movj(const std::vector<double> &xd) override;
    bool relj(const std::vector<double> &xd) override;
    bool movl(const std::vector<double> &xd) override;
    bool movv(const std::vector<double> &xdotd) override;
    bool gcmp() override;
    bool forc(const std::vector<double> &fd) override;
    bool stopControl() override;
    bool wait(double timeout) override;
    bool tool(const std::vector<double> &x) override;
    bool act(int command) override;
    void movi(const std::vector<double> &x) override;
    void pose(const std::vector<double> &x, double interval) override;
    void twist(const std::vector<double> &xdot) override;
    void wrench(const std::vector<double> &w) override;
    bool setParameter(int vocab, double value) override;
    bool getParameter(int vocab, double * value) override;
    bool setParameters(const std::map<int, double> & params) override;
    bool getParameters(std::map<int, double> & params) override;

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

private:
    bool handleRpcRunnableCmd(int vocab);
    bool handleRpcConsumerCmd(int vocab, const std::vector<double>& in);
    bool handleRpcFunctionCmd(int vocab, const std::vector<double>& in, std::vector<double>& out);

    void handleStreamingConsumerCmd(int vocab, const std::vector<double>& in);
    void handleStreamingBiConsumerCmd(int vocab, const std::vector<double>& in1, double in2);

    yarp::os::RpcClient rpcClient;
    yarp::os::BufferedPort<yarp::os::Bottle> fkInPort, commandPort;

    FkStreamResponder fkStreamResponder;
    double fkStreamTimeoutSecs;
};

} // namespace roboticslab

#endif // __CARTESIAN_CONTROL_CLIENT_HPP__

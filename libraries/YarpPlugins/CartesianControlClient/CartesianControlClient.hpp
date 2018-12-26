// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_CLIENT_HPP__
#define __CARTESIAN_CONTROL_CLIENT_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Semaphore.h>

#include <yarp/dev/Drivers.h>

#include <vector>

#include "ICartesianControl.h"

#define DEFAULT_CARTESIAN_LOCAL "/CartesianControl"
#define DEFAULT_CARTESIAN_REMOTE "/CartesianControl"

#define DEFAULT_FK_STREAM_TIMEOUT_SECS 0.5

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup CartesianControlClient
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

    FkStreamResponder() : now(0.0),
                          state(0)
    {}

    void onRead(yarp::os::Bottle& b);

    double getLastStatData(int *state, std::vector<double> &x);

protected:

    double now;
    int state;
    std::vector<double> x;

    yarp::os::Semaphore mutex;
};

/**
 * @ingroup CartesianControlClient
 * @brief The CartesianControlClient class implements ICartesianControl client side.
 */
class CartesianControlClient : public yarp::dev::DeviceDriver, public ICartesianControl
{
public:

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

    virtual bool wait(double timeout);

    virtual bool tool(const std::vector<double> &x);

    virtual void twist(const std::vector<double> &xdot);

    virtual void pose(const std::vector<double> &x, double interval);

    virtual void movi(const std::vector<double> &x);

    virtual bool setParameter(int vocab, double value);

    virtual bool getParameter(int vocab, double * value);

    virtual bool setParameters(const std::map<int, double> & params);

    virtual bool getParameters(std::map<int, double> & params);

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

    bool handleRpcRunnableCmd(int vocab);
    bool handleRpcConsumerCmd(int vocab, const std::vector<double>& in);
    bool handleRpcFunctionCmd(int vocab, const std::vector<double>& in, std::vector<double>& out);

    void handleStreamingConsumerCmd(int vocab, const std::vector<double>& in);
    void handleStreamingBiConsumerCmd(int vocab, const std::vector<double>& in1, double in2);

    yarp::os::RpcClient rpcClient;
    yarp::os::BufferedPort<yarp::os::Bottle> fkInPort, commandPort;

    FkStreamResponder fkStreamResponder;
    double fkStreamTimeoutSecs;

    bool fkStreamEnabled;
};

}  // namespace roboticslab

#endif  // __CARTESIAN_CONTROL_CLIENT_HPP__

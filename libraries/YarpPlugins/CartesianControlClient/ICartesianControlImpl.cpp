// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- ICartesianControl Related ------------------------------------

yarp::dev::ReturnValue CartesianControlClient::handleStreamingConsumerCmd(Streaming vocab, const std::vector<double> & in)
{
    auto & cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));

    for (auto i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    commandPort.write();
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::stat(std::vector<double> & x, State * state, double * timestamp)
{
    if (!fkInPort.isClosed())
    {
        if (!fkStreamResponder.getLastStatData(x, state, timestamp, m_fkStreamTimeoutSecs))
        {
            yCWarning(CCC) << "FK stream timeout, falling back to RPC request";
        }
        else
        {
            return yarp::dev::ReturnValue::return_code::return_value_ok;
        }
    }

    auto ret = rpcSender.stat();
    x = ret.x;

    if (state != nullptr)
    {
        *state = ret.state;
    }

    if (timestamp != nullptr)
    {
        *timestamp = ret.timestamp;
    }

    return ret.ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::inv(const std::vector<double> & xd, std::vector<double> & q)
{
    auto ret = rpcSender.inv(xd);
    q = ret.q;
    return ret.ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::movj(const std::vector<double> & xd)
{
    return rpcSender.movj(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::relj(const std::vector<double> & xd)
{
    return rpcSender.relj(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::movl(const std::vector<double> & xd)
{
    return rpcSender.movl(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::movv(const std::vector<double> & xdotd)
{
    return rpcSender.movv(xdotd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::gcmp()
{
    return rpcSender.gcmp();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::forc(const std::vector<double> & fd)
{
    return rpcSender.forc(fd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::stopControl()
{
    return rpcSender.stopControl();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::wait(double timeout)
{
    return rpcSender.wait(timeout);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::tool(const std::vector<double> & x)
{
    return rpcSender.tool(x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::act(Actuator command)
{
    return rpcSender.act(command);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::pose(const std::vector<double> & x)
{
    return handleStreamingConsumerCmd(Streaming::POSE, x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::twist(const std::vector<double> & xdot)
{
    return handleStreamingConsumerCmd(Streaming::TWIST, xdot);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::wrench(const std::vector<double> & w)
{
    return handleStreamingConsumerCmd(Streaming::WRENCH, w);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::setParameter(Config vocab, double value)
{
    return rpcSender.setParameter(vocab, value);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::getParameter(Config vocab, double * value)
{
    auto ret = rpcSender.getParameter(vocab);
    *value = ret.value;
    return ret.ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::setParameters(const std::map<Config, double> & params)
{
    return rpcSender.setParameters(params);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::getParameters(std::map<Config, double> & params)
{
    auto ret = rpcSender.getParameters();
    params = ret.params;
    return ret.ret;
}

// -----------------------------------------------------------------------------

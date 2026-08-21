// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- ICartesianControl Related ------------------------------------

void CartesianControlClient::handleStreamingConsumerCmd(Streaming vocab, const std::vector<double> & in)
{
    auto & cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));

    for (auto i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    commandPort.write();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::getState(std::vector<double> & x, State * state, double * timestamp)
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

    auto ret = rpcSender.getState();
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

yarp::dev::ReturnValue CartesianControlClient::solvePose(const std::vector<double> & xd, std::vector<double> & q)
{
    auto ret = rpcSender.solvePose(xd);
    q = ret.q;
    return ret.ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::moveJoint(const std::vector<double> & xd)
{
    return rpcSender.moveJoint(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::moveLinear(const std::vector<double> & xd)
{
    return rpcSender.moveLinear(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::moveVelocity(const std::vector<double> & xdotd)
{
    return rpcSender.moveVelocity(xdotd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::gravityCompensation()
{
    return rpcSender.gravityCompensation();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::forceControl(const std::vector<double> & fd)
{
    return rpcSender.forceControl(fd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::stopControl()
{
    return rpcSender.stopControl();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::changeTool(const std::vector<double> & x)
{
    return rpcSender.changeTool(x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::actuateTool(Actuator command)
{
    return rpcSender.actuateTool(command);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::pose(const std::vector<double> & x)
{
    return handleStreamingConsumerCmd(Streaming::POSE, x);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::twist(const std::vector<double> & xdot)
{
    return handleStreamingConsumerCmd(Streaming::TWIST, xdot);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::wrench(const std::vector<double> & w)
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

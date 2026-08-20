// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline bool checkSuccess(const yarp::os::Bottle & response)
    {
        return !response.get(0).isVocab32() || response.get(0).asVocab32() != static_cast<yarp::conf::vocab32_t>(ICartesianControl::Vocabs::FAILED);
    }

    inline void addValue(yarp::os::Bottle& b, yarp::conf::vocab32_t vocab, double value)
    {
        if (vocab == static_cast<yarp::conf::vocab32_t>(ICartesianControl::Config::FRAME) ||
            vocab == static_cast<yarp::conf::vocab32_t>(ICartesianControl::Config::STREAMING_CMD))
        {
            b.addVocab32(static_cast<yarp::conf::vocab32_t>(value));
        }
        else
        {
            b.addFloat64(value);
        }
    }

    inline double asValue(yarp::conf::vocab32_t vocab, const yarp::os::Value& v)
    {
        if (vocab == static_cast<yarp::conf::vocab32_t>(ICartesianControl::Config::FRAME) ||
            vocab == static_cast<yarp::conf::vocab32_t>(ICartesianControl::Config::STREAMING_CMD))
        {
            return v.asVocab32();
        }
        else
        {
            return v.asFloat64();
        }
    }
}

// ------------------- ICartesianControl Related ------------------------------------

bool CartesianControlClient::handleRpcRunnableCmd(RPC vocab)
{
    yarp::os::Bottle cmd, response;
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));
    rpcClient.write(cmd, response);
    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::handleRpcConsumerCmd(RPC vocab, const std::vector<double>& in)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));

    for (auto i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::handleRpcFunctionCmd(RPC vocab, const std::vector<double>& in, std::vector<double>& out)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));

    for (auto i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    out.resize(response.size());

    for (auto i = 0; i < response.size(); i++)
    {
        out[i] = response.get(i).asFloat64();
    }

    return true;
}

// -----------------------------------------------------------------------------

void CartesianControlClient::handleStreamingConsumerCmd(Streaming vocab, const std::vector<double>& in)
{
    yarp::os::Bottle& cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));

    for (auto i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    commandPort.write();
}

// -----------------------------------------------------------------------------

void CartesianControlClient::handleStreamingBiConsumerCmd(Streaming vocab, const std::vector<double>& in1, double in2)
{
    yarp::os::Bottle& cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));
    cmd.addFloat64(in2);

    for (auto i = 0; i < in1.size(); i++)
    {
        cmd.addFloat64(in1[i]);
    }

    commandPort.write();
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::stat(std::vector<double> &x, State * state, double * timestamp)
{
    if (!fkInPort.isClosed())
    {
        if (!fkStreamResponder.getLastStatData(x, state, timestamp, m_fkStreamTimeoutSecs))
        {
            yCWarning(CCC) << "FK stream timeout, falling back to RPC request";
        }
        else
        {
            return true;
        }
    }

    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(RPC::STAT));

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    if (state != nullptr)
    {
        *state = static_cast<State>(response.get(0).asVocab32());
    }

    x.resize(response.size() - 2);

    for (auto i = 0; i < x.size(); i++)
    {
        x[i] = response.get(i + 1).asFloat64();
    }

    if (timestamp != nullptr)
    {
        *timestamp = response.get(response.size() - 1).asFloat64();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    return handleRpcFunctionCmd(RPC::INV, xd, q);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::movj(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(RPC::MOVJ, xd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::relj(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(RPC::RELJ, xd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::movl(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(RPC::MOVL, xd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::movv(const std::vector<double> &xdotd)
{
    return handleRpcConsumerCmd(RPC::MOVV, xdotd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::gcmp()
{
    return handleRpcRunnableCmd(RPC::GCMP);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::forc(const std::vector<double> &fd)
{
    return handleRpcConsumerCmd(RPC::FORC, fd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::stopControl()
{
    return handleRpcRunnableCmd(RPC::STOP);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::wait(double timeout)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(RPC::WAIT));
    cmd.addFloat64(timeout);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::tool(const std::vector<double> &x)
{
    return handleRpcConsumerCmd(RPC::TOOL, x);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::act(Actuator command)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(RPC::ACT));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(command));

    rpcClient.write(cmd,response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::pose(const std::vector<double> &x)
{
    handleStreamingConsumerCmd(Streaming::POSE, x);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::twist(const std::vector<double> &xdot)
{
    handleStreamingConsumerCmd(Streaming::TWIST, xdot);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::wrench(const std::vector<double> &w)
{
    handleStreamingConsumerCmd(Streaming::WRENCH, w);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::setParameter(Config vocab, double value)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Vocabs::SET));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));
    addValue(cmd, static_cast<yarp::conf::vocab32_t>(vocab), value);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::getParameter(Config vocab, double * value)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Vocabs::GET));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    *value = asValue(static_cast<yarp::conf::vocab32_t>(vocab), response.get(0));

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::setParameters(const std::map<Config, double> & params)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Vocabs::SET));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Config::PARAMS));

    for (const auto & [key, value] : params)
    {
        yarp::os::Bottle & b = cmd.addList();
        b.addVocab32(static_cast<yarp::conf::vocab32_t>(key));
        addValue(b, static_cast<yarp::conf::vocab32_t>(key), value);
    }

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::getParameters(std::map<Config, double> & params)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Vocabs::GET));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Config::PARAMS));

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    for (int i = 0; i < response.size(); i++)
    {
        yarp::os::Bottle * b = response.get(i).asList();
        auto key = static_cast<Config>(b->get(0).asVocab32());
        double value = asValue(static_cast<yarp::conf::vocab32_t>(key), b->get(1));
        params.emplace(key, value);
    }

    return true;
}

// -----------------------------------------------------------------------------

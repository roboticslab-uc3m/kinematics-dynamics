// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline yarp::dev::ReturnValue checkSuccess(const yarp::os::Bottle & response)
    {
        return !response.get(0).isVocab32() || response.get(0).asVocab32() != static_cast<yarp::conf::vocab32_t>(ICartesianControl::Vocabs::FAILED)
            ? yarp::dev::ReturnValue::return_code::return_value_ok
            : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
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

yarp::dev::ReturnValue CartesianControlClient::handleRpcRunnableCmd(RPC vocab)
{
    yarp::os::Bottle cmd, response;
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));
    rpcClient.write(cmd, response);
    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::handleRpcConsumerCmd(RPC vocab, const std::vector<double>& in)
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

yarp::dev::ReturnValue CartesianControlClient::handleRpcFunctionCmd(RPC vocab, const std::vector<double>& in, std::vector<double>& out)
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
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    out.resize(response.size());

    for (auto i = 0; i < response.size(); i++)
    {
        out[i] = response.get(i).asFloat64();
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::handleStreamingConsumerCmd(Streaming vocab, const std::vector<double>& in)
{
    yarp::os::Bottle& cmd = commandPort.prepare();

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

yarp::dev::ReturnValue CartesianControlClient::handleStreamingBiConsumerCmd(Streaming vocab, const std::vector<double>& in1, double in2)
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
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::stat(std::vector<double> &x, State * state, double * timestamp)
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

    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(RPC::STAT));

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
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

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    return handleRpcFunctionCmd(RPC::INV, xd, q);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::movj(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(RPC::MOVJ, xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::relj(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(RPC::RELJ, xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::movl(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(RPC::MOVL, xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::movv(const std::vector<double> &xdotd)
{
    return handleRpcConsumerCmd(RPC::MOVV, xdotd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::gcmp()
{
    return handleRpcRunnableCmd(RPC::GCMP);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::forc(const std::vector<double> &fd)
{
    return handleRpcConsumerCmd(RPC::FORC, fd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::stopControl()
{
    return handleRpcRunnableCmd(RPC::STOP);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::wait(double timeout)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(RPC::WAIT));
    cmd.addFloat64(timeout);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::tool(const std::vector<double> &x)
{
    return handleRpcConsumerCmd(RPC::TOOL, x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::act(Actuator command)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(RPC::ACT));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(command));

    rpcClient.write(cmd,response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::pose(const std::vector<double> &x)
{
    return handleStreamingConsumerCmd(Streaming::POSE, x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::twist(const std::vector<double> &xdot)
{
    return handleStreamingConsumerCmd(Streaming::TWIST, xdot);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::wrench(const std::vector<double> &w)
{
    return handleStreamingConsumerCmd(Streaming::WRENCH, w);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::setParameter(Config vocab, double value)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Vocabs::SET));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));
    addValue(cmd, static_cast<yarp::conf::vocab32_t>(vocab), value);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::getParameter(Config vocab, double * value)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Vocabs::GET));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    *value = asValue(static_cast<yarp::conf::vocab32_t>(vocab), response.get(0));

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::setParameters(const std::map<Config, double> & params)
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

yarp::dev::ReturnValue CartesianControlClient::getParameters(std::map<Config, double> & params)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Vocabs::GET));
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(Config::PARAMS));

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    for (int i = 0; i < response.size(); i++)
    {
        yarp::os::Bottle * b = response.get(i).asList();
        auto key = static_cast<Config>(b->get(0).asVocab32());
        double value = asValue(static_cast<yarp::conf::vocab32_t>(key), b->get(1));
        params.emplace(key, value);
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

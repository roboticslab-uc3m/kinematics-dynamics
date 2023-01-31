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
        return !response.get(0).isVocab32() || response.get(0).asVocab32() != VOCAB_CC_FAILED;
    }

    inline void addValue(yarp::os::Bottle& b, int vocab, double value)
    {
        if (vocab == VOCAB_CC_CONFIG_FRAME || vocab == VOCAB_CC_CONFIG_STREAMING_CMD)
        {
            b.addVocab32(static_cast<yarp::conf::vocab32_t>(value));
        }
        else
        {
            b.addFloat64(value);
        }
    }

    inline double asValue(int vocab, const yarp::os::Value& v)
    {
        if (vocab == VOCAB_CC_CONFIG_FRAME || vocab == VOCAB_CC_CONFIG_STREAMING_CMD)
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

bool CartesianControlClient::handleRpcRunnableCmd(int vocab)
{
    yarp::os::Bottle cmd, response;
    cmd.addVocab32(vocab);
    rpcClient.write(cmd, response);
    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::handleRpcConsumerCmd(int vocab, const std::vector<double>& in)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(vocab);

    for (auto i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::handleRpcFunctionCmd(int vocab, const std::vector<double>& in, std::vector<double>& out)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(vocab);

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

void CartesianControlClient::handleStreamingConsumerCmd(int vocab, const std::vector<double>& in)
{
    yarp::os::Bottle& cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab32(vocab);

    for (auto i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    commandPort.write();
}

// -----------------------------------------------------------------------------

void CartesianControlClient::handleStreamingBiConsumerCmd(int vocab, const std::vector<double>& in1, double in2)
{
    yarp::os::Bottle& cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab32(vocab);
    cmd.addFloat64(in2);

    for (auto i = 0; i < in1.size(); i++)
    {
        cmd.addFloat64(in1[i]);
    }

    commandPort.write();
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::stat(std::vector<double> &x, int * state, double * timestamp)
{
    if (!fkInPort.isClosed())
    {
        if (!fkStreamResponder.getLastStatData(x, state, timestamp, fkStreamTimeoutSecs))
        {
            yCWarning(CCC) << "FK stream timeout, falling back to RPC request";
        }
        else
        {
            return true;
        }
    }

    yarp::os::Bottle cmd, response;

    cmd.addVocab32(VOCAB_CC_STAT);

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    if (state != 0)
    {
        *state = response.get(0).asVocab32();
    }

    x.resize(response.size() - 2);

    for (auto i = 0; i < x.size(); i++)
    {
        x[i] = response.get(i + 1).asFloat64();
    }

    if (timestamp != 0)
    {
        *timestamp = response.get(response.size() - 1).asFloat64();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    return handleRpcFunctionCmd(VOCAB_CC_INV, xd, q);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::movj(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(VOCAB_CC_MOVJ, xd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::relj(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(VOCAB_CC_RELJ, xd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::movl(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(VOCAB_CC_MOVL, xd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::movv(const std::vector<double> &xdotd)
{
    return handleRpcConsumerCmd(VOCAB_CC_MOVV, xdotd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::gcmp()
{
    return handleRpcRunnableCmd(VOCAB_CC_GCMP);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::forc(const std::vector<double> &fd)
{
    return handleRpcConsumerCmd(VOCAB_CC_FORC, fd);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::stopControl()
{
    return handleRpcRunnableCmd(VOCAB_CC_STOP);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::wait(double timeout)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(VOCAB_CC_WAIT);
    cmd.addFloat64(timeout);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::tool(const std::vector<double> &x)
{
    return handleRpcConsumerCmd(VOCAB_CC_TOOL, x);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::act(int command)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(VOCAB_CC_ACT);
    cmd.addVocab32(command);

    rpcClient.write(cmd,response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::twist(const std::vector<double> &xdot)
{
    handleStreamingConsumerCmd(VOCAB_CC_TWIST, xdot);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::pose(const std::vector<double> &x, double interval)
{
    handleStreamingBiConsumerCmd(VOCAB_CC_POSE, x, interval);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::movi(const std::vector<double> &x)
{
    handleStreamingConsumerCmd(VOCAB_CC_MOVI, x);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::wrench(const std::vector<double> &w)
{
    handleStreamingConsumerCmd(VOCAB_CC_WRENCH, w);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::setParameter(int vocab, double value)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(VOCAB_CC_SET);
    cmd.addVocab32(vocab);
    addValue(cmd, vocab, value);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::getParameter(int vocab, double * value)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(VOCAB_CC_GET);
    cmd.addVocab32(vocab);

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    *value = asValue(vocab, response.get(0));

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::setParameters(const std::map<int, double> & params)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(VOCAB_CC_SET);
    cmd.addVocab32(VOCAB_CC_CONFIG_PARAMS);

    for (std::map<int, double>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
        yarp::os::Bottle & b = cmd.addList();
        b.addVocab32(it->first);
        addValue(b, it->first, it->second);
    }

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::getParameters(std::map<int, double> & params)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab32(VOCAB_CC_GET);
    cmd.addVocab32(VOCAB_CC_CONFIG_PARAMS);

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    for (int i = 0; i < response.size(); i++)
    {
        yarp::os::Bottle * b = response.get(i).asList();
        int vocab = b->get(0).asVocab32();
        double value = asValue(vocab, b->get(1));
        params.emplace(vocab, value);
    }

    return true;
}

// -----------------------------------------------------------------------------

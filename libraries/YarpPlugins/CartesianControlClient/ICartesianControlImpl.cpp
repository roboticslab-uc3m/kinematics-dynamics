// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/Time.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

namespace
{
    inline bool checkSuccess(const yarp::os::Bottle & response)
    {
        return !response.get(0).isVocab() || response.get(0).asVocab() != VOCAB_CC_FAILED;
    }

    inline void addValue(yarp::os::Bottle& b, int vocab, double value)
    {
        if (vocab == VOCAB_CC_CONFIG_FRAME || vocab == VOCAB_CC_CONFIG_STREAMING)
        {
            b.addVocab(value);
        }
        else
        {
            b.addFloat64(value);
        }
    }

    inline double asValue(int vocab, const yarp::os::Value& v)
    {
        if (vocab == VOCAB_CC_CONFIG_FRAME || vocab == VOCAB_CC_CONFIG_STREAMING)
        {
            return v.asVocab();
        }
        else
        {
            return v.asFloat64();
        }
    }
}

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::CartesianControlClient::handleRpcRunnableCmd(int vocab)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(vocab);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::handleRpcConsumerCmd(int vocab, const std::vector<double>& in)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(vocab);

    for (size_t i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::handleRpcFunctionCmd(int vocab, const std::vector<double>& in, std::vector<double>& out)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(vocab);

    for (size_t i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    for (size_t i = 0; i < response.size(); i++)
    {
        out.push_back(response.get(i).asFloat64());
    }

    return true;
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::handleStreamingConsumerCmd(int vocab, const std::vector<double>& in)
{
    yarp::os::Bottle& cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab(vocab);

    for (size_t i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    commandPort.write();
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::handleStreamingBiConsumerCmd(int vocab, const std::vector<double>& in1, double in2)
{
    yarp::os::Bottle& cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab(vocab);
    cmd.addFloat64(in2);

    for (size_t i = 0; i < in1.size(); i++)
    {
        cmd.addFloat64(in1[i]);
    }

    commandPort.write();
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::stat(std::vector<double> &x, int * state, double * timestamp)
{
    if (!fkInPort.isClosed())
    {
        if (!fkStreamResponder.getLastStatData(x, state, timestamp, fkStreamTimeoutSecs))
        {
            CD_WARNING("FK stream timeout, falling back to RPC request.\n");
        }
        else
        {
            return true;
        }
    }

    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_STAT);

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    if (state != 0)
    {
        *state = response.get(0).asVocab();
    }

    x.resize(response.size() - 2);

    for (size_t i = 0; i < x.size(); i++)
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

bool roboticslab::CartesianControlClient::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    return handleRpcFunctionCmd(VOCAB_CC_INV, xd, q);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::movj(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(VOCAB_CC_MOVJ, xd);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::relj(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(VOCAB_CC_RELJ, xd);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::movl(const std::vector<double> &xd)
{
    return handleRpcConsumerCmd(VOCAB_CC_MOVL, xd);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::movv(const std::vector<double> &xdotd)
{
    return handleRpcConsumerCmd(VOCAB_CC_MOVV, xdotd);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::gcmp()
{
    return handleRpcRunnableCmd(VOCAB_CC_GCMP);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::forc(const std::vector<double> &td)
{
    return handleRpcConsumerCmd(VOCAB_CC_FORC, td);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::stopControl()
{
    return handleRpcRunnableCmd(VOCAB_CC_STOP);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::wait(double timeout)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_WAIT);
    cmd.addFloat64(timeout);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::tool(const std::vector<double> &x)
{
    return handleRpcConsumerCmd(VOCAB_CC_TOOL, x);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::twist(const std::vector<double> &xdot)
{
    handleStreamingConsumerCmd(VOCAB_CC_TWIST, xdot);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::pose(const std::vector<double> &x, double interval)
{
    handleStreamingBiConsumerCmd(VOCAB_CC_POSE, x, interval);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::movi(const std::vector<double> &x)
{
    handleStreamingConsumerCmd(VOCAB_CC_MOVI, x);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::setParameter(int vocab, double value)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_SET);
    cmd.addVocab(vocab);
    addValue(cmd, vocab, value);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::getParameter(int vocab, double * value)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_GET);
    cmd.addVocab(vocab);

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    *value = asValue(vocab, response.get(0));

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::setParameters(const std::map<int, double> & params)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_SET);
    cmd.addVocab(VOCAB_CC_CONFIG_PARAMS);

    for (std::map<int, double>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
        yarp::os::Bottle & b = cmd.addList();
        b.addVocab(it->first);
        addValue(b, it->first, it->second);
    }

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::getParameters(std::map<int, double> & params)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_GET);
    cmd.addVocab(VOCAB_CC_CONFIG_PARAMS);

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    for (int i = 0; i < response.size(); i++)
    {
        yarp::os::Bottle * b = response.get(i).asList();
        int vocab = b->get(0).asVocab();
        double value = asValue(vocab, b->get(1));
        std::pair<int, double> el(vocab, value);
        params.insert(el);
    }

    return true;
}

// -----------------------------------------------------------------------------

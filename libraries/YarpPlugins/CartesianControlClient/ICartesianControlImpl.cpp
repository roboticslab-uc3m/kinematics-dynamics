// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/conf/version.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

// -----------------------------------------------------------------------------

namespace
{
    inline bool checkSuccess(const yarp::os::Bottle & response)
    {
#if YARP_VERSION_MINOR >= 5
        return !response.get(0).isVocab32() || response.get(0).asVocab32() != VOCAB_CC_FAILED;
#else
        return !response.get(0).isVocab() || response.get(0).asVocab() != VOCAB_CC_FAILED;
#endif
    }

    inline void addValue(yarp::os::Bottle& b, int vocab, double value)
    {
        if (vocab == VOCAB_CC_CONFIG_FRAME || vocab == VOCAB_CC_CONFIG_STREAMING_CMD)
        {
#if YARP_VERSION_MINOR >= 5
            b.addVocab32(static_cast<yarp::conf::vocab32_t>(value));
#else
            b.addVocab(value);
#endif
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
#if YARP_VERSION_MINOR >= 5
            return v.asVocab32();
#else
            return v.asVocab();
#endif
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

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(vocab);
#else
    cmd.addVocab(vocab);
#endif

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::handleRpcConsumerCmd(int vocab, const std::vector<double>& in)
{
    yarp::os::Bottle cmd, response;

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(vocab);
#else
    cmd.addVocab(vocab);
#endif

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

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(vocab);
#else
    cmd.addVocab(vocab);
#endif

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
#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(vocab);
#else
    cmd.addVocab(vocab);
#endif

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
#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(vocab);
#else
    cmd.addVocab(vocab);
#endif
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
            yWarning() << "FK stream timeout, falling back to RPC request";
        }
        else
        {
            return true;
        }
    }

    yarp::os::Bottle cmd, response;

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(VOCAB_CC_STAT);
#else
    cmd.addVocab(VOCAB_CC_STAT);
#endif

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    if (state != 0)
    {
#if YARP_VERSION_MINOR >= 5
        *state = response.get(0).asVocab32();
#else
        *state = response.get(0).asVocab();
#endif
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

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(VOCAB_CC_WAIT);
#else
    cmd.addVocab(VOCAB_CC_WAIT);
#endif
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

bool roboticslab::CartesianControlClient::act(int command)
{
    yarp::os::Bottle cmd, response;

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(VOCAB_CC_ACT);
    cmd.addVocab32(command);
#else
    cmd.addVocab(VOCAB_CC_ACT);
    cmd.addVocab(command);
#endif

    rpcClient.write(cmd,response);

    return checkSuccess(response);
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

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(VOCAB_CC_SET);
    cmd.addVocab32(vocab);
#else
    cmd.addVocab(VOCAB_CC_SET);
    cmd.addVocab(vocab);
#endif
    addValue(cmd, vocab, value);

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::getParameter(int vocab, double * value)
{
    yarp::os::Bottle cmd, response;

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(VOCAB_CC_GET);
    cmd.addVocab32(vocab);
#else
    cmd.addVocab(VOCAB_CC_GET);
    cmd.addVocab(vocab);
#endif

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

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(VOCAB_CC_SET);
    cmd.addVocab32(VOCAB_CC_CONFIG_PARAMS);
#else
    cmd.addVocab(VOCAB_CC_SET);
    cmd.addVocab(VOCAB_CC_CONFIG_PARAMS);
#endif

    for (std::map<int, double>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
        yarp::os::Bottle & b = cmd.addList();
#if YARP_VERSION_MINOR >= 5
        b.addVocab32(it->first);
#else
        b.addVocab(it->first);
#endif
        addValue(b, it->first, it->second);
    }

    rpcClient.write(cmd, response);

    return checkSuccess(response);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::getParameters(std::map<int, double> & params)
{
    yarp::os::Bottle cmd, response;

#if YARP_VERSION_MINOR >= 5
    cmd.addVocab32(VOCAB_CC_GET);
    cmd.addVocab32(VOCAB_CC_CONFIG_PARAMS);
#else
    cmd.addVocab(VOCAB_CC_GET);
    cmd.addVocab(VOCAB_CC_CONFIG_PARAMS);
#endif

    rpcClient.write(cmd, response);

    if (!checkSuccess(response))
    {
        return false;
    }

    for (int i = 0; i < response.size(); i++)
    {
        yarp::os::Bottle * b = response.get(i).asList();
#if YARP_VERSION_MINOR >= 5
        int vocab = b->get(0).asVocab32();
#else
        int vocab = b->get(0).asVocab();
#endif
        double value = asValue(vocab, b->get(1));
        std::pair<int, double> el(vocab, value);
        params.insert(el);
    }

    return true;
}

// -----------------------------------------------------------------------------

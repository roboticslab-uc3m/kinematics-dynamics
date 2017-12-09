// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/Time.h>

#include <ColorDebug.hpp>

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::CartesianControlClient::handleRpcRunnableCmd(int vocab)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(vocab);

    rpcClient.write(cmd, response);

    if (response.get(0).isVocab() && response.get(0).asVocab() == VOCAB_FAILED)
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::handleRpcConsumerCmd(int vocab, const std::vector<double>& in)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(vocab);

    for (size_t i = 0; i < in.size(); i++)
    {
        cmd.addDouble(in[i]);
    }

    rpcClient.write(cmd, response);

    if (response.get(0).isVocab() && response.get(0).asVocab() == VOCAB_FAILED)
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::handleRpcFunctionCmd(int vocab, const std::vector<double>& in, std::vector<double>& out)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(vocab);

    for (size_t i = 0; i < in.size(); i++)
    {
        cmd.addDouble(in[i]);
    }

    rpcClient.write(cmd, response);

    if (response.get(0).isVocab() && response.get(0).asVocab() == VOCAB_FAILED)
    {
        return false;
    }

    for (size_t i = 0; i < response.size(); i++)
    {
        out.push_back(response.get(i).asDouble());
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
        cmd.addDouble(in[i]);
    }

    commandPort.write(true);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::handleStreamingBiConsumerCmd(int vocab, const std::vector<double>& in1, double in2)
{
    yarp::os::Bottle& cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab(vocab);
    cmd.addDouble(in2);

    for (size_t i = 0; i < in1.size(); i++)
    {
        cmd.addDouble(in1[i]);
    }

    commandPort.write(true);
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::stat(int &state, std::vector<double> &x)
{
    if (fkStreamEnabled)
    {
        double localArrivalTime = fkStreamResponder.getLastStatData(&state, x);

        if (yarp::os::Time::now() - localArrivalTime <= FK_STREAM_TIMEOUT_SECS)
        {
            return true;
        }

        CD_WARNING("FK stream timeout, sending RPC request.\n");
    }

    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_STAT);

    rpcClient.write(cmd, response);

    if (response.get(0).isVocab() && response.get(0).asVocab() == VOCAB_FAILED)
    {
        return false;
    }

    state = response.get(0).asVocab();
    x.resize(response.size() - 1);

    for (size_t i = 0; i < x.size(); i++)
    {
        x[i] = response.get(i + 1).asDouble();
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

bool roboticslab::CartesianControlClient::tool(const std::vector<double> &x)
{
    return handleRpcConsumerCmd(VOCAB_CC_TOOL, x);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::fwd(const std::vector<double> &rot, double step)
{
    handleStreamingBiConsumerCmd(VOCAB_CC_FWD, rot, step);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::bkwd(const std::vector<double> &rot, double step)
{
    handleStreamingBiConsumerCmd(VOCAB_CC_BKWD, rot, step);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::rot(const std::vector<double> &rot)
{
    handleStreamingConsumerCmd(VOCAB_CC_ROT, rot);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::pan(const std::vector<double> &transl)
{
    handleStreamingConsumerCmd(VOCAB_CC_PAN, transl);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::vmos(const std::vector<double> &xdot)
{
    handleStreamingConsumerCmd(VOCAB_CC_VMOS, xdot);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::eff(const std::vector<double> &xdotee)
{
    handleStreamingConsumerCmd(VOCAB_CC_EFF, xdotee);
}

// -----------------------------------------------------------------------------

void roboticslab::CartesianControlClient::pose(const std::vector<double> &x, double interval)
{
    handleStreamingBiConsumerCmd(VOCAB_CC_POSE, x, interval);
}

// -----------------------------------------------------------------------------

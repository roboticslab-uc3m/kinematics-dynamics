// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <sstream>

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

namespace
{
    inline bool isGroupParam(const yarp::os::Bottle& in)
    {
        return in.size() > 1 && in.get(1).asVocab() == VOCAB_CC_CONFIG_PARAMS;
    }

    inline void addValue(yarp::os::Bottle& b, int vocab, double value)
    {
        if (vocab == VOCAB_CC_CONFIG_FRAME || vocab == VOCAB_CC_CONFIG_STREAMING)
        {
            b.addVocab(value);
        }
        else
        {
            b.addDouble(value);
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
            return v.asDouble();
        }
    }
}

// ------------------- RpcResponder Related ------------------------------------

bool roboticslab::RpcResponder::respond(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    // process data "in", prepare "out"
    CD_DEBUG("Got: %s\n", in.toString().c_str());

    switch (in.get(0).asVocab())
    {
    case VOCAB_CC_STAT:
        return handleStatMsg(in, out);
    case VOCAB_CC_INV:
        return handleFunctionCmdMsg(in, out, &ICartesianControl::inv);
    case VOCAB_CC_MOVJ:
        return handleConsumerCmdMsg(in, out, &ICartesianControl::movj);
    case VOCAB_CC_RELJ:
        return handleConsumerCmdMsg(in, out, &ICartesianControl::relj);
    case VOCAB_CC_MOVL:
        return handleConsumerCmdMsg(in, out, &ICartesianControl::movl);
    case VOCAB_CC_MOVV:
        return handleConsumerCmdMsg(in, out, &ICartesianControl::movv);
    case VOCAB_CC_GCMP:
        return handleRunnableCmdMsg(in, out, &ICartesianControl::gcmp);
    case VOCAB_CC_FORC:
        return handleConsumerCmdMsg(in, out, &ICartesianControl::forc);
    case VOCAB_CC_STOP:
        return handleRunnableCmdMsg(in, out, &ICartesianControl::stopControl);
    case VOCAB_CC_WAIT:
        return handleWaitMsg(in, out);
    case VOCAB_CC_TOOL:
        return handleConsumerCmdMsg(in, out, &ICartesianControl::tool);
    case VOCAB_CC_SET:
        return isGroupParam(in) ? handleParameterSetterGroup(in, out) : handleParameterSetter(in, out);
    case VOCAB_CC_GET:
        return isGroupParam(in) ? handleParameterGetterGroup(in, out) : handleParameterGetter(in, out);
    default:
        return DeviceResponder::respond(in, out);
    }
}

// -----------------------------------------------------------------------------

void roboticslab::RpcResponder::makeUsage()
{
    std::stringstream ss;

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_STAT) << "]";
    addUsage(ss.str().c_str(), "get current position in cartesian space");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_INV) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "accept desired position in cartesian space, return result in joint space");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_MOVJ) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "joint move to desired position (absolute coordinates in cartesian space)");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_RELJ) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "joint move to desired position (relative coordinates in cartesian space)");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_MOVL) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "linear move to desired position (absolute coordinates in cartesian space)");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_MOVV) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "velocity move using supplied vector (cartesian space)");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_GCMP) << "]";
    addUsage(ss.str().c_str(), "enable gravity compensation");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_FORC) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "enable torque control, apply input forces (cartesian space)");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_STOP) << "]";
    addUsage(ss.str().c_str(), "stop control");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_WAIT) << "] timeout";
    addUsage(ss.str().c_str(), "wait until completion with timeout (optional, 0.0 means no timeout)");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_TOOL) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "append fixed link to end effector");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_SET) << "] vocab value";
    addUsage(ss.str().c_str(), "set configuration parameter");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_GET) << "] vocab";
    addUsage(ss.str().c_str(), "get configuration parameter");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_SET) << "] [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_PARAMS) << "] (vocab value) ...";
    addUsage(ss.str().c_str(), "set multiple configuration parameters");
    ss.str("");

    ss << "[" << yarp::os::Vocab::decode(VOCAB_CC_GET) << "] [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_PARAMS) << "]";
    addUsage(ss.str().c_str(), "get all configuration parameters");
    ss.str("");

    ss << "... [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_GAIN) << "] value";
    addUsage(ss.str().c_str(), "(config param) controller gain");
    ss.str("");

    ss << "... [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_MAX_JOINT_VEL) << "] value";
    addUsage(ss.str().c_str(), "(config param) maximum joint velocity");
    ss.str("");

    ss << "... [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_TRAJ_DURATION) << "] value";
    addUsage(ss.str().c_str(), "(config param) trajectory duration");
    ss.str("");

    ss << "... [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_CMC_RATE) << "] value";
    addUsage(ss.str().c_str(), "(config param) CMC rate [ms]");
    ss.str("");

    ss << "... [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_WAIT_PERIOD) << "] value";
    addUsage(ss.str().c_str(), "(config param) check period of 'wait' command [ms]");
    ss.str("");

    ss << "... [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_FRAME) << "] [" << yarp::os::Vocab::decode(ICartesianSolver::BASE_FRAME) << "]";
    addUsage(ss.str().c_str(), "(config param) reference frame (base)");
    ss.str("");

    ss << "... [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_FRAME) << "] [" << yarp::os::Vocab::decode(ICartesianSolver::TCP_FRAME) << "]";
    addUsage(ss.str().c_str(), "(config param) reference frame (TCP)");
    ss.str("");

    std::stringstream ss_cmd;
    ss_cmd << "(config param) preset streaming command, available:";
    ss_cmd << " [" << yarp::os::Vocab::decode(VOCAB_CC_TWIST) << "]";
    ss_cmd << " [" << yarp::os::Vocab::decode(VOCAB_CC_POSE) << "]";
    ss_cmd << " [" << yarp::os::Vocab::decode(VOCAB_CC_MOVI) << "]";

    ss << "... [" << yarp::os::Vocab::decode(VOCAB_CC_CONFIG_STREAMING) << "] vocab";
    addUsage(ss.str().c_str(), ss_cmd.str().c_str());

    ss.str("");
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleStatMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    std::vector<double> x;
    int state;

    if (iCartesianControl->stat(state, x))
    {
        if (!transformOutgoingData(x))
        {
            out.addVocab(VOCAB_CC_FAILED);
            return false;
        }

        out.addVocab(state);

        for (size_t i = 0; i < x.size(); i++)
        {
            out.addDouble(x[i]);
        }

        return true;
    }
    else
    {
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleWaitMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    bool res;

    if (in.size() > 1)
    {
        double timeout = in.get(1).asDouble();
        res = iCartesianControl->wait(timeout);
    }
    else
    {
        res = iCartesianControl->wait();
    }

    if (!res)
    {
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }

    out.addVocab(VOCAB_CC_OK);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleRunnableCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, RunnableFun cmd)
{
    if ((iCartesianControl->*cmd)())
    {
        out.addVocab(VOCAB_CC_OK);
        return true;
    }
    else
    {
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleConsumerCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, ConsumerFun cmd)
{
    if (in.size() > 1)
    {
        std::vector<double> vin;

        for (size_t i = 1; i < in.size(); i++)
        {
            vin.push_back(in.get(i).asDouble());
        }

        if (!transformIncomingData(vin) || !(iCartesianControl->*cmd)(vin))
        {
            out.addVocab(VOCAB_CC_FAILED);
            return false;
        }

        out.addVocab(VOCAB_CC_OK);
        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleFunctionCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, FunctionFun cmd)
{
    if (in.size() > 1)
    {
        std::vector<double> vin, vout;

        for (size_t i = 1; i < in.size(); i++)
        {
            vin.push_back(in.get(i).asDouble());
        }

        if (!transformIncomingData(vin) || !(iCartesianControl->*cmd)(vin, vout))
        {
            out.addVocab(VOCAB_CC_FAILED);
            return false;
        }

        for (size_t i = 0; i < vout.size(); i++)
        {
            out.addDouble(vout[i]);
        }

        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleParameterSetter(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 2)
    {
        int vocab = in.get(1).asVocab();
        double value = asValue(vocab, in.get(2));

        if (!iCartesianControl->setParameter(vocab, value))
        {
            out.addVocab(VOCAB_CC_FAILED);
            return false;
        }

        out.addVocab(VOCAB_CC_OK);
        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleParameterGetter(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
        int vocab = in.get(1).asVocab();
        double value;

        if (!iCartesianControl->getParameter(vocab, &value))
        {
            out.addVocab(VOCAB_CC_FAILED);
            return false;
        }

        addValue(out, vocab, value);
        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleParameterSetterGroup(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 2)
    {
        std::map<int, double> params;

        for (int i = 2; i < in.size(); i++)
        {
            if (!in.get(i).isList() || in.get(i).asList()->size() != 2)
            {
                CD_ERROR("bottle format error\n");
                out.addVocab(VOCAB_CC_FAILED);
                return false;
            }

            yarp::os::Bottle * b = in.get(i).asList();
            int vocab = b->get(0).asVocab();
            double value = asValue(vocab, b->get(1));
            std::pair<int, double> el(vocab, value);
            params.insert(el);
        }

        if (!iCartesianControl->setParameters(params))
        {
            out.addVocab(VOCAB_CC_FAILED);
            return false;
        }

        out.addVocab(VOCAB_CC_OK);
        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleParameterGetterGroup(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() == 2)
    {
        std::map<int, double> params;

        if (!iCartesianControl->getParameters(params))
        {
            out.addVocab(VOCAB_CC_FAILED);
            return false;
        }

        for (std::map<int, double>::const_iterator it = params.begin(); it != params.end(); ++it)
        {
            yarp::os::Bottle & b = out.addList();
            b.addVocab(it->first);
            addValue(b, it->first, it->second);
        }

        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_CC_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcTransformResponder::transformIncomingData(std::vector<double>& vin)
{
    return KinRepresentation::encodePose(vin, vin, KinRepresentation::CARTESIAN, orient, KinRepresentation::DEGREES);
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcTransformResponder::transformOutgoingData(std::vector<double>& vout)
{
    return KinRepresentation::decodePose(vout, vout, KinRepresentation::CARTESIAN, orient, KinRepresentation::DEGREES);
}

// -----------------------------------------------------------------------------

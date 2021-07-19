// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <sstream>

#include <yarp/conf/version.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline bool isGroupParam(const yarp::os::Bottle& in)
    {
#if YARP_VERSION_MINOR >= 5
        return in.size() > 1 && in.get(1).asVocab32() == VOCAB_CC_CONFIG_PARAMS;
#else
        return in.size() > 1 && in.get(1).asVocab() == VOCAB_CC_CONFIG_PARAMS;
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

// ------------------- RpcResponder Related ------------------------------------

bool RpcResponder::respond(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
#if YARP_VERSION_MINOR >= 5
    switch (in.get(0).asVocab32())
#else
    switch (in.get(0).asVocab())
#endif
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
    case VOCAB_CC_ACT:
        return handleActMsg(in, out);
    case VOCAB_CC_SET:
        return isGroupParam(in) ? handleParameterSetterGroup(in, out) : handleParameterSetter(in, out);
    case VOCAB_CC_GET:
        return isGroupParam(in) ? handleParameterGetterGroup(in, out) : handleParameterGetter(in, out);
    default:
        return DeviceResponder::respond(in, out);
    }
}

// -----------------------------------------------------------------------------

void RpcResponder::makeUsage()
{
#if YARP_VERSION_MINOR >= 5
    namespace Vocab = yarp::os::Vocab32;
#else
    namespace Vocab = yarp::os::Vocab;
#endif

    std::stringstream ss;

    ss << "[" << Vocab::decode(VOCAB_CC_STAT) << "]";
    addUsage(ss.str().c_str(), "get controller state, current position in cartesian space and encoder acquisition timestamp");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_INV) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "accept desired position in cartesian space, return result in joint space");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_MOVJ) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "joint move to desired position (absolute coordinates in cartesian space)");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_RELJ) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "joint move to desired position (relative coordinates in cartesian space)");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_MOVL) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "linear move to desired position (absolute coordinates in cartesian space)");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_MOVV) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "velocity move using supplied vector (cartesian space)");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_GCMP) << "]";
    addUsage(ss.str().c_str(), "enable gravity compensation");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_FORC) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "enable torque control, apply input forces (cartesian space)");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_STOP) << "]";
    addUsage(ss.str().c_str(), "stop control");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_WAIT) << "] timeout";
    addUsage(ss.str().c_str(), "wait until completion with timeout (optional, 0.0 means no timeout)");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_TOOL) << "] coord1 coord2 ...";
    addUsage(ss.str().c_str(), "append fixed link to end effector");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_ACT) << "] vocab";
    addUsage(ss.str().c_str(), "actuate tool using selected command vocab");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_SET) << "] vocab value";
    addUsage(ss.str().c_str(), "set configuration parameter");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_GET) << "] vocab";
    addUsage(ss.str().c_str(), "get configuration parameter");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_SET) << "] [" << Vocab::decode(VOCAB_CC_CONFIG_PARAMS) << "] (vocab value) ...";
    addUsage(ss.str().c_str(), "set multiple configuration parameters");
    ss.str("");

    ss << "[" << Vocab::decode(VOCAB_CC_GET) << "] [" << Vocab::decode(VOCAB_CC_CONFIG_PARAMS) << "]";
    addUsage(ss.str().c_str(), "get all configuration parameters");
    ss.str("");

    ss << "... [" << Vocab::decode(VOCAB_CC_CONFIG_GAIN) << "] value";
    addUsage(ss.str().c_str(), "(config param) controller gain");
    ss.str("");

    ss << "... [" << Vocab::decode(VOCAB_CC_CONFIG_TRAJ_DURATION) << "] value";
    addUsage(ss.str().c_str(), "(config param) trajectory duration");
    ss.str("");

    ss << "... [" << Vocab::decode(VOCAB_CC_CONFIG_CMC_PERIOD) << "] value";
    addUsage(ss.str().c_str(), "(config param) CMC period [ms]");
    ss.str("");

    std::stringstream ss_wait;
    ss_wait << "(config param) check period of [" << Vocab::decode(VOCAB_CC_WAIT) << "] command [ms]";

    ss << "... [" << Vocab::decode(VOCAB_CC_CONFIG_WAIT_PERIOD) << "] value";
    addUsage(ss.str().c_str(), ss_wait.str().c_str());
    ss.str("");

    std::stringstream ss_frame;
    ss_frame << "(config param) reference frame, available (base/TCP):";
    ss_frame << " [" << Vocab::decode(ICartesianSolver::BASE_FRAME) << "]";
    ss_frame << " [" << Vocab::decode(ICartesianSolver::TCP_FRAME) << "]";

    ss << "... [" << Vocab::decode(VOCAB_CC_CONFIG_FRAME) << "] vocab";
    addUsage(ss.str().c_str(), ss_frame.str().c_str());
    ss.str("");

    std::stringstream ss_cmd;
    ss_cmd << "(config param) preset streaming command, available:";
    ss_cmd << " [" << Vocab::decode(VOCAB_CC_TWIST) << "]";
    ss_cmd << " [" << Vocab::decode(VOCAB_CC_POSE) << "]";
    ss_cmd << " [" << Vocab::decode(VOCAB_CC_MOVI) << "]";

    ss << "... [" << Vocab::decode(VOCAB_CC_CONFIG_STREAMING_CMD) << "] vocab";
    addUsage(ss.str().c_str(), ss_cmd.str().c_str());
    ss.str("");
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleStatMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    std::vector<double> x;
    int state;
    double timestamp;

    if (iCartesianControl->stat(x, &state, &timestamp))
    {
        if (!transformOutgoingData(x))
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_FAILED);
#else
            out.addVocab(VOCAB_CC_FAILED);
#endif
            return false;
        }

#if YARP_VERSION_MINOR >= 5
        out.addVocab32(state);
#else
        out.addVocab(state);
#endif

        for (size_t i = 0; i < x.size(); i++)
        {
            out.addFloat64(x[i]);
        }

        out.addFloat64(timestamp);

        return true;
    }
    else
    {
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleWaitMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    bool res;

    if (in.size() > 1)
    {
        double timeout = in.get(1).asFloat64();
        res = iCartesianControl->wait(timeout);
    }
    else
    {
        res = iCartesianControl->wait();
    }

    if (!res)
    {
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }

#if YARP_VERSION_MINOR >= 5
    out.addVocab32(VOCAB_CC_OK);
#else
    out.addVocab(VOCAB_CC_OK);
#endif
    return true;
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleActMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
#if YARP_VERSION_MINOR >= 5
        int commandCode = in.get(1).asVocab32();
#else
        int commandCode = in.get(1).asVocab();
#endif

        if (iCartesianControl->act(commandCode))
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_OK);
#else
            out.addVocab(VOCAB_CC_OK);
#endif
            return true;
        }
        else
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_FAILED);
#else
            out.addVocab(VOCAB_CC_FAILED);
#endif
            return false;
        }
    }
    else
    {
        yCError(CCS) << "Size error:" << in.size();
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleRunnableCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, RunnableFun cmd)
{
    if ((iCartesianControl->*cmd)())
    {
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_OK);
#else
        out.addVocab(VOCAB_CC_OK);
#endif
        return true;
    }
    else
    {
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleConsumerCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, ConsumerFun cmd)
{
    if (in.size() > 1)
    {
        std::vector<double> vin;

        for (size_t i = 1; i < in.size(); i++)
        {
            vin.push_back(in.get(i).asFloat64());
        }

        if (!transformIncomingData(vin) || !(iCartesianControl->*cmd)(vin))
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_FAILED);
#else
            out.addVocab(VOCAB_CC_FAILED);
#endif
            return false;
        }

#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_OK);
#else
        out.addVocab(VOCAB_CC_OK);
#endif
        return true;
    }
    else
    {
        yCError(CCS) << "Size error:" << in.size();
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleFunctionCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, FunctionFun cmd)
{
    if (in.size() > 1)
    {
        std::vector<double> vin, vout;

        for (size_t i = 1; i < in.size(); i++)
        {
            vin.push_back(in.get(i).asFloat64());
        }

        if (!transformIncomingData(vin) || !(iCartesianControl->*cmd)(vin, vout))
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_FAILED);
#else
            out.addVocab(VOCAB_CC_FAILED);
#endif
            return false;
        }

        for (size_t i = 0; i < vout.size(); i++)
        {
            out.addFloat64(vout[i]);
        }

        return true;
    }
    else
    {
        yCError(CCS) << "Size error:" << in.size();
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleParameterSetter(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 2)
    {
#if YARP_VERSION_MINOR >= 5
        int vocab = in.get(1).asVocab32();
#else
        int vocab = in.get(1).asVocab();
#endif
        double value = asValue(vocab, in.get(2));

        if (!iCartesianControl->setParameter(vocab, value))
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_FAILED);
#else
            out.addVocab(VOCAB_CC_FAILED);
#endif
            return false;
        }

#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_OK);
#else
        out.addVocab(VOCAB_CC_OK);
#endif
        return true;
    }
    else
    {
        yCError(CCS) << "Size error:" << in.size();
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleParameterGetter(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
#if YARP_VERSION_MINOR >= 5
        int vocab = in.get(1).asVocab32();
#else
        int vocab = in.get(1).asVocab();
#endif
        double value;

        if (!iCartesianControl->getParameter(vocab, &value))
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_FAILED);
#else
            out.addVocab(VOCAB_CC_FAILED);
#endif
            return false;
        }

        addValue(out, vocab, value);
        return true;
    }
    else
    {
        yCError(CCS) << "Size error:" << in.size();
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleParameterSetterGroup(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 2)
    {
        std::map<int, double> params;

        for (int i = 2; i < in.size(); i++)
        {
            if (!in.get(i).isList() || in.get(i).asList()->size() != 2)
            {
                yCError(CCS) << "Bottle format error";
#if YARP_VERSION_MINOR >= 5
                out.addVocab32(VOCAB_CC_FAILED);
#else
                out.addVocab(VOCAB_CC_FAILED);
#endif
                return false;
            }

            yarp::os::Bottle * b = in.get(i).asList();
#if YARP_VERSION_MINOR >= 5
            int vocab = b->get(0).asVocab32();
#else
            int vocab = b->get(0).asVocab();
#endif
            double value = asValue(vocab, b->get(1));
            params.emplace(std::make_pair(vocab, value));
        }

        if (!iCartesianControl->setParameters(params))
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_FAILED);
#else
            out.addVocab(VOCAB_CC_FAILED);
#endif
            return false;
        }

#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_OK);
#else
        out.addVocab(VOCAB_CC_OK);
#endif
        return true;
    }
    else
    {
        yCError(CCS) << "Size error:" << in.size();
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcResponder::handleParameterGetterGroup(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() == 2)
    {
        std::map<int, double> params;

        if (!iCartesianControl->getParameters(params))
        {
#if YARP_VERSION_MINOR >= 5
            out.addVocab32(VOCAB_CC_FAILED);
#else
            out.addVocab(VOCAB_CC_FAILED);
#endif
            return false;
        }

        for (const auto & it : params)
        {
            yarp::os::Bottle & b = out.addList();
#if YARP_VERSION_MINOR >= 5
            b.addVocab32(it.first);
#else
            b.addVocab(it.first);
#endif
            addValue(b, it.first, it.second);
        }

        return true;
    }
    else
    {
        yCError(CCS) << "Size error:" << in.size();
#if YARP_VERSION_MINOR >= 5
        out.addVocab32(VOCAB_CC_FAILED);
#else
        out.addVocab(VOCAB_CC_FAILED);
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool RpcTransformResponder::transformIncomingData(std::vector<double>& vin)
{
    return KinRepresentation::encodePose(vin, vin, coord, orient, units);
}

// -----------------------------------------------------------------------------

bool RpcTransformResponder::transformOutgoingData(std::vector<double>& vout)
{
    return KinRepresentation::decodePose(vout, vout, coord, orient, units);
}

// -----------------------------------------------------------------------------

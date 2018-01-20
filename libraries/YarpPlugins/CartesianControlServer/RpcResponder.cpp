// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <ColorDebug.hpp>

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
    case VOCAB_CC_TOOL:
        return handleConsumerCmdMsg(in, out, &ICartesianControl::tool);
    case VOCAB_CC_CONFIG_SET:
        return handleParameterSetter(in, out);
    case VOCAB_CC_CONFIG_GET:
        return handleParameterGetter(in, out);
    default:
        return DeviceResponder::respond(in, out);
    }
}

// -----------------------------------------------------------------------------

void roboticslab::RpcResponder::makeUsage()
{
    addUsage("[stat]", "get current position in cartesian space");
    addUsage("[inv] coord1 coord2 ...", "accept desired position in cartesian space, return result in joint space");
    addUsage("[movj] coord1 coord2 ...", "joint move to desired position (absolute coordinates in cartesian space)");
    addUsage("[relj] coord1 coord2 ...", "joint move to desired position (relative coordinates in cartesian space)");
    addUsage("[movl] coord1 coord2 ...", "linear move to desired position (absolute coordinates in cartesian space)");
    addUsage("[movv] coord1 coord2 ...", "velocity move using supplied vector (cartesian space)");
    addUsage("[gcmp]", "enable gravity compensation");
    addUsage("[forc] coord1 coord2 ...", "enable torque control, apply input forces (cartesian space)");
    addUsage("[stop]", "stop control");
    addUsage("[tool] coord1 coord2 ...", "append fixed link to end effector");
    addUsage("[cps] vocab value", "set configuration parameter");
    addUsage("[cpg] vocab", "get configuration parameter");
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
            out.addVocab(VOCAB_FAILED);
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
        out.addVocab(VOCAB_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleRunnableCmdMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out, RunnableFun cmd)
{
    if ((iCartesianControl->*cmd)())
    {
        out.addVocab(VOCAB_OK);
        return true;
    }
    else
    {
        out.addVocab(VOCAB_FAILED);
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
            out.addVocab(VOCAB_FAILED);
            return false;
        }

        out.addVocab(VOCAB_OK);
        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_FAILED);
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
            out.addVocab(VOCAB_FAILED);
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
        out.addVocab(VOCAB_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleParameterSetter(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 2)
    {
        int vocab = in.get(1).asVocab();
        double value = in.get(2).asDouble();

        if (!iCartesianControl->setParameter(vocab, value))
        {
            out.addVocab(VOCAB_FAILED);
            return false;
        }

        out.addVocab(VOCAB_OK);
        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_FAILED);
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
            out.addVocab(VOCAB_FAILED);
            return false;
        }

        out.addDouble(value);
        return true;
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_FAILED);
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

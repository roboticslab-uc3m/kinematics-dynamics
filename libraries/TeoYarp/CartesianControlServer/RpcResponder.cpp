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
    case VOCAB_CC_GCMP:
        return handleRunnableCmdMsg(in, out, &ICartesianControl::gcmp);
    case VOCAB_CC_FORC:
        return handleConsumerCmdMsg(in, out, &ICartesianControl::forc);
    case VOCAB_CC_STOP:
        return handleRunnableCmdMsg(in, out, &ICartesianControl::stopControl);
    default:
        return DeviceResponder::respond(in, out);
    }
}

// -----------------------------------------------------------------------------

void roboticslab::RpcResponder::makeUsage()
{
    // shadows DeviceResponder::makeUsage(), which was already called by the base constructor
    addUsage("[stat]", "get current position in cartesian space");
    addUsage("[inv] $fCoord1 $fCoord2 ...", "accept desired position in cartesian space, return result in joint space");
    addUsage("[movj] $fCoord1 $fCoord2 ...", "joint move to desired position (absolute coordinates in cartesian space)");
    addUsage("[relj] $fCoord1 $fCoord2 ...", "joint move to desired position (relative coordinates in cartesian space)");
    addUsage("[movl] $fCoord1 $fCoord2 ...", "linear move to desired position (absolute coordinates in cartesian space)");
    addUsage("[movv] $fCoord1 $fCoord2 ...", "velocity move using supplied vector (cartesian space)");
    addUsage("[gcmp]", "enable gravity compensation");
    addUsage("[forc] $fCoord1 $fCoord2 ...", "enable torque control, apply input forces (cartesian space)");
    addUsage("[stop]", "stop control");
}

// -----------------------------------------------------------------------------

bool roboticslab::RpcResponder::handleStatMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    std::vector<double> x;
    int state;

    if (iCartesianControl->stat(state, x))
    {
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

        if ((iCartesianControl->*cmd)(vin))
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

        if ((iCartesianControl->*cmd)(vin, vout))
        {
            for (size_t i = 0; i < vout.size(); i++)
            {
                out.addDouble(vout[i]);
            }

            return true;
        }
        else
        {
            out.addVocab(VOCAB_FAILED);
            return false;
        }
    }
    else
    {
        CD_ERROR("size error\n");
        out.addVocab(VOCAB_FAILED);
        return false;
    }
}

// -----------------------------------------------------------------------------

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
        return handleInvMsg(in, out);
    case VOCAB_CC_MOVJ:
        return handleMovjMsg(in, out);
    case VOCAB_CC_RELJ:
        return handleReljMsg(in, out);
    case VOCAB_CC_MOVL:
        return handleMovlMsg(in, out);
    case VOCAB_CC_GCMP:
        return handleGcmpMsg(in, out);
    case VOCAB_CC_FORC:
        return handleForcMsg(in, out);
    case VOCAB_CC_STOP:
        return handleStopMsg(in, out);
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

bool roboticslab::RpcResponder::handleInvMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
        std::vector<double> xd, q;

        for (size_t i = 1; i < in.size(); i++)
        {
            xd.push_back(in.get(i).asDouble());
        }

        if (iCartesianControl->inv(xd,q))
        {
            for (size_t i = 0; i < q.size(); i++)
            {
                out.addDouble(q[i]);
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

bool roboticslab::RpcResponder::handleMovjMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
        std::vector<double> xd;

        for (size_t i = 1; i < in.size(); i++)
        {
            xd.push_back(in.get(i).asDouble());
        }

        if (iCartesianControl->movj(xd))
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

bool roboticslab::RpcResponder::handleReljMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
        std::vector<double> xd;

        for (size_t i = 1; i < in.size(); i++)
        {
            xd.push_back(in.get(i).asDouble());
        }

        if (iCartesianControl->relj(xd))
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

bool roboticslab::RpcResponder::handleMovlMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
        std::vector<double> xd;

        for (size_t i = 1; i < in.size(); i++)
        {
            xd.push_back(in.get(i).asDouble());
        }

        if (iCartesianControl->movl(xd))
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

bool roboticslab::RpcResponder::handleMovvMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
        std::vector<double> xdotd;

        for (size_t i = 1; i < in.size(); i++)
        {
            xdotd.push_back(in.get(i).asDouble());
        }

        if (iCartesianControl->movv(xdotd))
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

bool roboticslab::RpcResponder::handleGcmpMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (iCartesianControl->gcmp())
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

bool roboticslab::RpcResponder::handleForcMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (in.size() > 1)
    {
        std::vector<double> td;

        for (size_t i = 1; i < in.size(); i++)
        {
            td.push_back(in.get(i).asDouble());
        }

        if (iCartesianControl->forc(td))
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

bool roboticslab::RpcResponder::handleStopMsg(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if (iCartesianControl->stopControl())
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

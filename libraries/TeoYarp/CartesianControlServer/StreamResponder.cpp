// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <ColorDebug.hpp>

// ------------------- StreamResponder Related ------------------------------------

void roboticslab::StreamResponder::onRead(yarp::os::Bottle& b)
{
    CD_DEBUG("Got: %s\n", b.toString().c_str());

    switch (b.get(0).asVocab())
    {
    case VOCAB_CC_FWD:
        handleBiConsumerCmdMsg(b, &ICartesianControl::fwd);
        break;
    case VOCAB_CC_BKWD:
        handleBiConsumerCmdMsg(b, &ICartesianControl::bkwd);
        break;
    case VOCAB_CC_ROT:
        handleConsumerCmdMsg(b, &ICartesianControl::rot);
        break;
    case VOCAB_CC_VMOS:
        handleConsumerCmdMsg(b, &ICartesianControl::vmos);
        break;
    case VOCAB_CC_POSE:
        handleBiConsumerCmdMsg(b, &ICartesianControl::pose);
        break;
    default:
        CD_ERROR("command not recognized\n");
        break;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::StreamResponder::handleConsumerCmdMsg(const yarp::os::Bottle& in, ConsumerFun cmd)
{
    if (in.size() > 1)
    {
        std::vector<double> v;

        for (size_t i = 1; i < in.size(); i++)
        {
            v.push_back(in.get(i).asDouble());
        }

        if (!(iCartesianControl->*cmd)(v))
        {
            CD_ERROR("command failed\n");
        }
    }
    else
    {
        CD_ERROR("size error\n");
    }
}

// -----------------------------------------------------------------------------

void roboticslab::StreamResponder::handleBiConsumerCmdMsg(const yarp::os::Bottle& in, BiConsumerFun cmd)
{
    if (in.size() > 2)
    {
        double d = in.get(1).asDouble();
        std::vector<double> v;

        for (size_t i = 2; i < in.size(); i++)
        {
            v.push_back(in.get(i).asDouble());
        }

        if (!(iCartesianControl->*cmd)(v, d))
        {
            CD_ERROR("command failed\n");
        }
    }
    else
    {
        CD_ERROR("size error\n");
    }
}

// -----------------------------------------------------------------------------

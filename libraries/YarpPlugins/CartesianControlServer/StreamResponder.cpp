// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <vector>

#include <ColorDebug.h>

// ------------------- StreamResponder Related ------------------------------------

void roboticslab::StreamResponder::onRead(yarp::os::Bottle& b)
{
    CD_DEBUG("Got: %s\n", b.toString().c_str());

    switch (b.get(0).asVocab())
    {
    case VOCAB_CC_TWIST:
        handleConsumerCmdMsg(b, &ICartesianControl::twist);
        break;
    case VOCAB_CC_POSE:
        handleBiConsumerCmdMsg(b, &ICartesianControl::pose);
        break;
    case VOCAB_CC_MOVI:
        handleConsumerCmdMsg(b, &ICartesianControl::movi);
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
            v.push_back(in.get(i).asFloat64());
        }

        (iCartesianControl->*cmd)(v);
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
        double d = in.get(1).asFloat64();
        std::vector<double> v;

        for (size_t i = 2; i < in.size(); i++)
        {
            v.push_back(in.get(i).asFloat64());
        }

        (iCartesianControl->*cmd)(v, d);
    }
    else
    {
        CD_ERROR("size error\n");
    }
}

// -----------------------------------------------------------------------------

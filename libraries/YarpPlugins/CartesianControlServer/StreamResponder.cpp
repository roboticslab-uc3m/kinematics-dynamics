// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <vector>

#include <yarp/conf/version.h>
#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- StreamResponder Related ------------------------------------

void StreamResponder::onRead(yarp::os::Bottle& b)
{
    yCDebug(CCS, "Got: %s", b.toString().c_str());

#if YARP_VERSION_MINOR >= 5
    switch (b.get(0).asVocab32())
#else
    switch (b.get(0).asVocab())
#endif
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
        yCError(CCS) << "Command not recognized:" << b.get(0).toString();
        break;
    }
}

// -----------------------------------------------------------------------------

void StreamResponder::handleConsumerCmdMsg(const yarp::os::Bottle& in, ConsumerFun cmd)
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
        yCError(CCS) << "Size error:" << in.size();
    }
}

// -----------------------------------------------------------------------------

void StreamResponder::handleBiConsumerCmdMsg(const yarp::os::Bottle& in, BiConsumerFun cmd)
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
        yCError(CCS) << "Size error:" << in.size();
    }
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

void FkStreamResponder::onRead(yarp::os::Bottle & b)
{
    if (b.size() != 10)
    {
        yCWarningThrottle(CCC, 1.0) << "Received FK stream message with incorrect size (should be 10):" << b.size();
        return;
    }

    std::lock_guard lock(mtx);

    localArrivalTime = yarp::os::SystemClock::nowSystem();
    mode = b.get(0).asVocab32();
    x.resize(6);

    for (size_t i = 0; i < x.size(); i++)
    {
        x[i] = b.get(i + 1).asFloat64();
    }

    timestamp = b.get(7).asFloat64();
    progress = b.get(8).asFloat32();
    success = b.get(9).asInt8() != 0;
}

// -----------------------------------------------------------------------------

bool FkStreamResponder::getLastStateData(ICartesianControl::ControllerState & state, const double timeout)
{
    std::lock_guard lock(mtx);

    state.x = x;
    state.mode = static_cast<ICartesianControl::Mode>(mode);
    state.timestamp = timestamp;
    state.progress = progress;
    state.success = success;

    return yarp::os::SystemClock::nowSystem() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------

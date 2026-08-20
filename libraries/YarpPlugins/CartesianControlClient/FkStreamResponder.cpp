// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/SystemClock.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

FkStreamResponder::FkStreamResponder()
    : localArrivalTime(0.0),
      state(0),
      timestamp(0.0)
{}

// -----------------------------------------------------------------------------

void FkStreamResponder::onRead(yarp::os::Bottle & b)
{
    std::lock_guard lock(mtx);

    localArrivalTime = yarp::os::SystemClock::nowSystem();
    state = b.get(0).asVocab32();
    x.resize(b.size() - 2);

    for (size_t i = 0; i < x.size(); i++)
    {
        x[i] = b.get(i + 1).asFloat64();
    }

    timestamp = b.get(b.size() - 1).asFloat64();
}

// -----------------------------------------------------------------------------

bool FkStreamResponder::getLastStatData(std::vector<double> & x, ICartesianControl::State * state, double * timestamp, const double timeout)
{
    std::lock_guard lock(mtx);

    x = this->x;

    if (state != nullptr)
    {
        *state = static_cast<ICartesianControl::State>(this->state);
    }

    if (timestamp != nullptr)
    {
        *timestamp = this->timestamp;
    }

    return yarp::os::SystemClock::nowSystem() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------

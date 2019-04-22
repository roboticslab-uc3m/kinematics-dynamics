// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/Time.h>

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
    mutex.wait();

    localArrivalTime = yarp::os::Time::now();
    state = b.get(0).asVocab();
    x.resize(b.size() - 2);

    for (size_t i = 0; i < x.size(); i++)
    {
        x[i] = b.get(i + 1).asDouble();
    }

    timestamp = b.get(b.size() - 1).asDouble();

    mutex.post();
}

// -----------------------------------------------------------------------------

bool FkStreamResponder::getLastStatData(std::vector<double> &x, int *state, double *timestamp, const double timeout)
{
    double now = yarp::os::Time::now();
    double localArrivalTime;

    mutex.wait();

    localArrivalTime = this->localArrivalTime;
    x = this->x;

    if (state != 0)
    {
        *state = this->state;
    }

    if (timestamp != 0)
    {
        *timestamp = this->timestamp;
    }

    mutex.post();

    return now - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/Time.h>

// -----------------------------------------------------------------------------

roboticslab::FkStreamResponder::FkStreamResponder()
    : localArrivalTime(0.0),
      state(0)
{}

// -----------------------------------------------------------------------------

void roboticslab::FkStreamResponder::onRead(yarp::os::Bottle & b)
{
    mutex.wait();

    localArrivalTime = yarp::os::Time::now();
    state = b.get(0).asVocab();
    x.resize(b.size() - 1);

    for (size_t i = 0; i < x.size(); i++)
    {
        x[i] = b.get(i + 1).asDouble();
    }

    mutex.post();
}

// -----------------------------------------------------------------------------

bool roboticslab::FkStreamResponder::getLastStatData(std::vector<double> &x, int *state, const double timeout)
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

    mutex.post();

    return now - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------

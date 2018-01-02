// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/Time.h>

// ------------------- DeviceDriver Related ------------------------------------

void roboticslab::FkStreamResponder::onRead(yarp::os::Bottle & b)
{
    mutex.wait();

    now = yarp::os::Time::now();
    state = b.get(0).asVocab();
    x.resize(b.size() - 1);

    for (size_t i = 0; i < x.size(); i++)
    {
        x[i] = b.get(i + 1).asDouble();
    }

    mutex.post();
}

// -----------------------------------------------------------------------------

double roboticslab::FkStreamResponder::getLastStatData(int *state, std::vector<double> &x)
{
    double localArrivalTime;

    mutex.wait();

    *state = this->state;
    x = this->x;
    localArrivalTime = now;

    mutex.post();

    return localArrivalTime;
}

// -----------------------------------------------------------------------------

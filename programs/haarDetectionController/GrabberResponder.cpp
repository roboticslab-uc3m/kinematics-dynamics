// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GrabberResponder.hpp"

#include <cmath>
#include <vector>

#include <ColorDebug.hpp>

namespace
{
    const double STEP_X = 0.0001;
    const double STEP_Y = 0.0001;

    // just in case, not used by now
    const double DEADBAND_X_PX = 0.0;
    const double DEADBAND_Y_PX = 0.0;
}

void roboticslab::GrabberResponder::onRead(yarp::os::Bottle &b)
{
    CD_DEBUG("Got: %s\n", b.toString().c_str());

    if (b.size() != 2)
    {
        CD_WARNING("Wrong data size: %d (expected: 2).\n", b.size());
        return;
    }

    const double input_x = b.get(0).asDouble();
    const double input_y = b.get(1).asDouble();

    double target_x = 0.0;
    double target_y = 0.0;

    bool move = false;

    if (std::abs(input_x) > DEADBAND_X_PX)
    {
        target_x = input_x * STEP_X;
        move = true;
    }

    if (std::abs(input_y) > DEADBAND_Y_PX)
    {
        target_y = input_y * STEP_Y;
        move = true;
    }

    if (move)
    {
        std::vector<double> transl(3);
        transl[0] = target_x;
        transl[1] = -target_y;  // inverted for AMOR

        iCartesianControl->pan(transl);

        isStopped = false;
    }
    else if (!isStopped)
    {
        iCartesianControl->stopControl();
        isStopped = true;
    }
    else
    {
        isStopped = true;
    }
}

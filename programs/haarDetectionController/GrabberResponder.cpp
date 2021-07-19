// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GrabberResponder.hpp"

#include <cmath>
#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr double STEP_X = 0.0001;
constexpr double STEP_Y = 0.0001;
constexpr double STEP_Z = 0.01;

// just in case, not used by now
constexpr double DEADBAND_X_PX = 0.0;
constexpr double DEADBAND_Y_PX = 0.0;

void GrabberResponder::onRead(yarp::os::Bottle &b)
{
    yCDebug(HDC) << "Got:" << b.toString();

    if (b.size() != 2)
    {
        yCWarning(HDC) << "Wrong data size:" << b.size() << "(expected: 2)";
        return;
    }

    const double input_x = b.get(0).asFloat64();
    const double input_y = b.get(1).asFloat64();

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
        std::vector<double> xdot(6);
        xdot[0] = target_x;
        xdot[1] = -target_y;  // inverted for AMOR
        xdot[2] = noApproach ? 0.0 : STEP_Z;

        iCartesianControl->twist(xdot);

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

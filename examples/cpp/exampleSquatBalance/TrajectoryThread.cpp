// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryThread.hpp"

#include <vector>

#include <yarp/os/Time.h>

bool TrajectoryThread::threadInit()
{
    startTime = yarp::os::Time::now();
    return true;
}

void TrajectoryThread::run()
{
    double movementTime = yarp::os::Time::now() - startTime;

    std::vector<double> position;
    iCartTrajectory->getPosition(movementTime, position);

    iCartesianControl->movi(position);
}

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearTrajectoryThread.hpp"

#include <yarp/os/Time.h>

#include "KdlTrajectory.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

LinearTrajectoryThread::LinearTrajectoryThread(int period, ICartesianControl * _iCartesianControl)
    : yarp::os::RateThread(period),
      iCartesianControl(_iCartesianControl),
      iCartesianTrajectory(new KdlTrajectory),
      startTime(0)
{}

LinearTrajectoryThread::~LinearTrajectoryThread()
{
    delete iCartesianTrajectory;
    iCartesianTrajectory = 0;
}

bool LinearTrajectoryThread::configure(const std::vector<double> & vels)
{
    int state;
    std::vector<double> x;

    if (!iCartesianControl->stat(state, x))
    {
        CD_ERROR("stat failed.\n");
        return false;
    }

    bool ok = true;

    mutex.lock();

    ok = ok && iCartesianTrajectory->destroy(); // discard previous state
    ok = ok && iCartesianTrajectory->addWaypoint(x, vels);
    ok = ok && iCartesianTrajectory->configurePath(ICartesianTrajectory::LINE);
    ok = ok && iCartesianTrajectory->configureVelocityProfile(ICartesianTrajectory::RECTANGULAR);
    ok = ok && iCartesianTrajectory->create();

    if (ok)
    {
        startTime = yarp::os::Time::now();
    }
    else
    {
        CD_ERROR("Unable to create trajectory.\n");
    }

    mutex.unlock();

    return ok;
}

void LinearTrajectoryThread::run()
{
    std::vector<double> position;

    mutex.lock();
    bool ok = iCartesianTrajectory->getPosition(yarp::os::Time::now() - startTime, position);
    mutex.unlock();

    if (ok)
    {
        iCartesianControl->movi(position);
    }
}

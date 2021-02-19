// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearTrajectoryThread.hpp"

#include <algorithm>
#include <functional>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "KdlTrajectory.hpp"

using namespace roboticslab;

LinearTrajectoryThread::LinearTrajectoryThread(int _period, ICartesianControl * _iCartesianControl)
    : yarp::os::PeriodicThread(_period * 0.001),
      period(_period),
      iCartesianControl(_iCartesianControl),
      iCartesianTrajectory(new KdlTrajectory),
      startTime(0.0),
      usingStreamingCommandConfig(false),
      usingTcpFrame(false)
{}

LinearTrajectoryThread::~LinearTrajectoryThread()
{
    delete iCartesianTrajectory;
    iCartesianTrajectory = 0;
}

bool LinearTrajectoryThread::checkStreamingConfig()
{
    std::map<int, double> params;

    if (!iCartesianControl->getParameters(params))
    {
        yWarning() << "getParameters failed";
        return false;
    }

    usingStreamingCommandConfig = params.find(VOCAB_CC_CONFIG_STREAMING_CMD) != params.end();

    return true;
}

bool LinearTrajectoryThread::configure(const std::vector<double> & vels)
{
    if (usingStreamingCommandConfig && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        yWarning() << "Unable to preset streaming command";
        return false;
    }

    if (usingTcpFrame)
    {
        std::vector<double> deltaX(vels.size());
        std::transform(vels.begin(), vels.end(), deltaX.begin(), std::bind1st(std::multiplies<double>(), period / 1000.0));

        mtx.lock();
        this->deltaX = deltaX;
        mtx.unlock();

        return true;
    }

    std::vector<double> x;

    if (!iCartesianControl->stat(x))
    {
        yError() << "stat failed";
        return false;
    }

    bool ok = true;

    mtx.lock();

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
        yError() << "Unable to create trajectory";
    }

    mtx.unlock();

    return ok;
}

void LinearTrajectoryThread::run()
{
    if (usingTcpFrame)
    {
        std::vector<double> deltaX;

        mtx.lock();
        deltaX = this->deltaX;
        mtx.unlock();

        iCartesianControl->movi(deltaX);
    }
    else
    {
        std::vector<double> position;

        mtx.lock();
        bool ok = iCartesianTrajectory->getPosition(yarp::os::Time::now() - startTime, position);
        mtx.unlock();

        if (ok)
        {
            iCartesianControl->movi(position);
        }
    }
}

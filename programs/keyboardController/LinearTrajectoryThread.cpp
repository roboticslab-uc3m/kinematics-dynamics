// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearTrajectoryThread.hpp"

#include <algorithm>
#include <functional>

#include <yarp/os/Time.h>

#include "KdlTrajectory.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

LinearTrajectoryThread::LinearTrajectoryThread(int _period, ICartesianControl * _iCartesianControl)
    : yarp::os::RateThread(_period),
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
        CD_WARNING("getParameters failed.\n");
        return false;
    }

    usingStreamingCommandConfig = params.find(VOCAB_CC_CONFIG_STREAMING) != params.end();

    return true;
}

bool LinearTrajectoryThread::configure(const std::vector<double> & vels)
{
    if (usingStreamingCommandConfig && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING, VOCAB_CC_MOVI))
    {
        CD_WARNING("Unable to preset streaming command.\n");
        return false;
    }

    if (usingTcpFrame)
    {
        std::vector<double> deltaX(vels.size());
        std::transform(vels.begin(), vels.end(), deltaX.begin(), std::bind1st(std::multiplies<double>(), period / 1000.0));

        mutex.lock();
        this->deltaX = deltaX;
        mutex.unlock();

        return true;
    }

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
    if (usingTcpFrame)
    {
        std::vector<double> deltaX;

        mutex.lock();
        deltaX = this->deltaX;
        mutex.unlock();

        iCartesianControl->movi(deltaX);
    }
    else
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
}

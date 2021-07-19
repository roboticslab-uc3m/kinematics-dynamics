// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearTrajectoryThread.hpp"

#include <algorithm>
#include <functional>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_rect.hpp>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

LinearTrajectoryThread::LinearTrajectoryThread(int _period, ICartesianControl * _iCartesianControl)
#if YARP_VERSION_MINOR >= 5
    : yarp::os::PeriodicThread(_period * 0.001, yarp::os::PeriodicThreadClock::Absolute),
#else
    : yarp::os::PeriodicThread(_period * 0.001),
#endif
      period(_period),
      iCartesianControl(_iCartesianControl),
      trajectory(nullptr),
      startTime(0.0),
      usingStreamingCommandConfig(false),
      usingTcpFrame(false)
{}

LinearTrajectoryThread::~LinearTrajectoryThread()
{
    delete trajectory;
    trajectory = nullptr;
}

bool LinearTrajectoryThread::checkStreamingConfig()
{
    std::map<int, double> params;

    if (!iCartesianControl->getParameters(params))
    {
        yCWarning(KC) << "getParameters failed";
        return false;
    }

    usingStreamingCommandConfig = params.find(VOCAB_CC_CONFIG_STREAMING_CMD) != params.end();

    return true;
}

bool LinearTrajectoryThread::configure(const std::vector<double> & vels)
{
    if (usingStreamingCommandConfig && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        yCWarning(KC) << "Unable to preset streaming command";
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
        yCError(KC) << "stat failed";
        return false;
    }

    KDL::Frame H = KdlVectorConverter::vectorToFrame(x);
    KDL::Twist tw = KdlVectorConverter::vectorToTwist(vels);

    mtx.lock();

    if (trajectory)
    {
        // discard previous state
        delete trajectory;
        trajectory = nullptr;
    }

    KDL::Path * path = new KDL::Path_Line(H, tw, new KDL::RotationalInterpolation_SingleAxis(), 1.0);
    KDL::VelocityProfile * profile = new KDL::VelocityProfile_Rectangular(10.0);
    profile->SetProfileDuration(0.0, 10.0, 10.0 / path->PathLength());
    trajectory = new KDL::Trajectory_Segment(path, profile);
    startTime = yarp::os::Time::now();

    mtx.unlock();

    return true;
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
        mtx.lock();
        KDL::Frame H = trajectory->Pos(yarp::os::Time::now() - startTime);
        mtx.unlock();

        std::vector<double> position = KdlVectorConverter::frameToVector(H);
        iCartesianControl->movi(position);
    }
}

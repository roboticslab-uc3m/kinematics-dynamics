// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_THREAD_HPP__
#define __TRAJECTORY_THREAD_HPP__

#include <yarp/conf/version.h>

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>

#include <kdl/trajectory.hpp>

#include <ScrewTheoryIkProblem.hpp>
#include <ConfigurationSelector.hpp>

class TrajectoryThread : public yarp::os::PeriodicThread
{
public:
    TrajectoryThread(yarp::dev::IEncoders * iEncoders, yarp::dev::IPositionDirect * iPosDirect,
                     roboticslab::ScrewTheoryIkProblem * ikProblem,
                     roboticslab::ConfigurationSelector * ikConfig,
                     KDL::Trajectory * trajectory,
                     int period)
#if YARP_VERSION_MINOR >= 5
        : yarp::os::PeriodicThread(period * 0.001, yarp::os::PeriodicThreadClock::Absolute),
#else
        : yarp::os::PeriodicThread(period * 0.001),
#endif
          iEncoders(iEncoders),
          iPosDirect(iPosDirect),
          ikProblem(ikProblem),
          ikConfig(ikConfig),
          trajectory(trajectory),
          axes(0),
          startTime(0)
    {}

protected:
    virtual bool threadInit();
    virtual void run();

private:
    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IPositionDirect * iPosDirect;
    roboticslab::ScrewTheoryIkProblem * ikProblem;
    roboticslab::ConfigurationSelector * ikConfig;
    KDL::Trajectory * trajectory;
    int axes;
    double startTime;
};

#endif  // __TRAJECTORY_THREAD_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_THREAD_HPP__
#define __TRAJECTORY_THREAD_HPP__

#include <yarp/os/PeriodicThread.h>

#include <ICartesianControl.h>
#include <ICartesianTrajectory.hpp>

namespace rl = roboticslab;

class TrajectoryThread : public yarp::os::PeriodicThread
{
public:
    TrajectoryThread(rl::ICartesianControl * iCartesianControl, rl::ICartesianTrajectory * iCartTrajectory, int period)
        : yarp::os::PeriodicThread(period * 0.001),
          iCartesianControl(iCartesianControl),
          iCartTrajectory(iCartTrajectory),
          startTime(0.0)
    {}

protected:
    virtual bool threadInit();
    virtual void run();

private:
    rl::ICartesianControl * iCartesianControl;
    rl::ICartesianTrajectory * iCartTrajectory;
    double startTime;
};

#endif  // __TRAJECTORY_THREAD_HPP__

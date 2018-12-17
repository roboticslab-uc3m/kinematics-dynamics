// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_THREAD_HPP__
#define __TRAJECTORY_THREAD_HPP__

#include <yarp/os/RateThread.h>

#include <yarp/dev/api.h> // upstream bug
#include <yarp/dev/IPositionDirect.h>

#include <ScrewTheoryIkProblem.hpp>
#include <ICartesianTrajectory.hpp>

class TrajectoryThread : public yarp::os::RateThread
{
public:
    TrajectoryThread(yarp::dev::IPositionDirect * iPosDirect, roboticslab::ScrewTheoryIkProblem * ikProblem, roboticslab::ICartesianTrajectory * iCartTrajectory, int period)
        : yarp::os::RateThread(period),
          iPosDirect(iPosDirect),
          ikProblem(ikProblem),
          iCartTrajectory(iCartTrajectory),
          startTime(0)
    {}

protected:
    virtual bool threadInit();
    virtual void run();

private:
    yarp::dev::IPositionDirect * iPosDirect;
    roboticslab::ScrewTheoryIkProblem * ikProblem;
    roboticslab::ICartesianTrajectory * iCartTrajectory;
    double startTime;
};

#endif  // __TRAJECTORY_THREAD_HPP__

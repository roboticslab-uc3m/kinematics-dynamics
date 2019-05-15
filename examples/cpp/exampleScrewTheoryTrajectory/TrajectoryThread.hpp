// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_THREAD_HPP__
#define __TRAJECTORY_THREAD_HPP__

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/api.h> // upstream bug in the IPositionDirect API
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>

#include <ScrewTheoryIkProblem.hpp>
#include <ConfigurationSelector.hpp>
#include <ICartesianTrajectory.hpp>

class TrajectoryThread : public yarp::os::PeriodicThread
{
public:
    TrajectoryThread(yarp::dev::IEncoders * iEncoders, yarp::dev::IPositionDirect * iPosDirect,
            roboticslab::ScrewTheoryIkProblem * ikProblem,
            roboticslab::ConfigurationSelector * ikConfig,
            roboticslab::ICartesianTrajectory * iCartTrajectory,
            int period)
        : yarp::os::PeriodicThread(period * 0.001),
          iEncoders(iEncoders),
          iPosDirect(iPosDirect),
          ikProblem(ikProblem),
          ikConfig(ikConfig),
          iCartTrajectory(iCartTrajectory),
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
    roboticslab::ICartesianTrajectory * iCartTrajectory;
    int axes;
    double startTime;
};

#endif  // __TRAJECTORY_THREAD_HPP__

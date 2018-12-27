// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_TRAJECTORY_THREAD_HPP__
#define __LINEAR_TRAJECTORY_THREAD_HPP__

#include <vector>

#include <yarp/os/Mutex.h>
#include <yarp/os/RateThread.h>

#include "ICartesianControl.h"
#include "ICartesianTrajectory.hpp"

namespace roboticslab
{

/**
 * @ingroup keyboardController
 *
 * @brief Periodic thread that encapsulates a linear trajectory
 */
class LinearTrajectoryThread : public yarp::os::RateThread
{
public:
    LinearTrajectoryThread(int period, ICartesianControl * iCartesianControl);
    ~LinearTrajectoryThread();
    bool configure(const std::vector<double> & vels);

protected:
    virtual void run();

private:
    ICartesianControl * iCartesianControl;
    ICartesianTrajectory * iCartesianTrajectory;
    double startTime;
    mutable yarp::os::Mutex mutex;
};

}  // namespace roboticslab

#endif  // __LINEAR_TRAJECTORY_THREAD_HPP__

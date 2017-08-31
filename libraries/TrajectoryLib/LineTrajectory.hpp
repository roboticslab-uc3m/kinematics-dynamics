// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINE_TRAJECTORY_HPP__
#define __LINE_TRAJECTORY_HPP__

#include <string>
#include <vector>

#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>

#include "Trajectory.hpp"

#define DEFAULT_MAXVEL 7.5      // unit/s
#define DEFAULT_MAXACC 0.2      // unit/s^2
#define DEFAULT_DURATION 10.0

namespace roboticslab
{

/**
 * @ingroup TrajectoryLib
 * @brief Implements a basic line trajectory.
 */
class LineTrajectory : public Trajectory
{
public:

    LineTrajectory();

    virtual bool getX(const double movementTime, std::vector<double>& x);
    virtual bool getXdot(const double movementTime, std::vector<double>& xdot);

    bool newLine(const std::vector<double> &src, const std::vector<double> &dest);
    bool deleteLine();

private:

    KDL::Trajectory_Segment* currentTrajectory;
    KDL::RotationalInterpolation_SingleAxis* _orient;

};

}  // namespace roboticslab

#endif  // __LINE_TRAJECTORY_HPP__

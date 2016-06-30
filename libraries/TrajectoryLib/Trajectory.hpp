// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

#include <vector>

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>

namespace teo
{

class Trajectory
{
public:

    bool getX(const double movementTime, std::vector<double>& desiredX);

    bool getXdot(const double movementTime, std::vector<double>& desiredXdot);

    bool generateLine(const std::vector<double> &src, const std::vector<double> &dest);

private:

    KDL::Trajectory_Segment* currentTrajectory;
    KDL::RotationalInterpolation_SingleAxis* _orient;

};

}  // namespace teo

#endif  // __TRAJECTORY_HPP__

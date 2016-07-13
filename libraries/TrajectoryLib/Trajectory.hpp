// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

#include <vector>

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>

#include "ColorDebug.hpp"
#include "KdlVectorConverter.hpp"

#define DEFAULT_MAXVEL 7.5      // unit/s
#define DEFAULT_MAXACC 0.2      // unit/s^2
#define DEFAULT_DURATION 10

namespace teo
{

/**
 * @ingroup teo_libraries
 * \defgroup TrajectoryLib
 *
 * @brief Contains classes related to trajectories.
 */

/**
 * @ingroup TrajectoryLib
 * @brief Implements a basic trajectory.
 */

class Trajectory : public KdlVectorConverter
{
public:

    Trajectory();

    /** Cartesian position of the trajectory at movementTime */
    bool getX(const double movementTime, std::vector<double>& x);

    /** Cartesian velocity of the trajectory at movementTime */
    bool getXdot(const double movementTime, std::vector<double>& xdot);

    bool newLine(const std::vector<double> &src, const std::vector<double> &dest);
    bool deleteLine();

private:

    KDL::Trajectory_Segment* currentTrajectory;
    KDL::RotationalInterpolation_SingleAxis* _orient;

};

}  // namespace teo

#endif  // __TRAJECTORY_HPP__

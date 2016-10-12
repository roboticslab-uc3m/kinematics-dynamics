// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

#include <vector>

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

class Trajectory
{
public:

    /** Cartesian position of the trajectory at movementTime */
    virtual bool getX(const double movementTime, std::vector<double>& x) = 0;

    /** Cartesian velocity of the trajectory at movementTime */
    virtual bool getXdot(const double movementTime, std::vector<double>& xdot) = 0;

};

}  // namespace teo

#endif  // __TRAJECTORY_HPP__

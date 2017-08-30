// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

#include <vector>

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-libraries
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

    /**
     * @brief Cartesian position of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param x 6-element vector describing a position in cartesian space; first
     * three elements denote translation (meters), last three denote rotation in
     * scaled axis-angle representation (radians).
     *
     * @return true on success, false otherwise
     */
    virtual bool getX(const double movementTime, std::vector<double>& x) = 0;

    /**
     * @brief Cartesian velocity of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param xdot 6-element vector describing a velocity in cartesian space; first
     * three elements denote translational velocity (meters/second), last three
     * denote angular velocity (radians/second).
     *
     * @return true on success, false otherwise
     */
    virtual bool getXdot(const double movementTime, std::vector<double>& xdot) = 0;

    /** Destructor */
    virtual ~Trajectory() {}
};

}  // namespace roboticslab

#endif  // __TRAJECTORY_HPP__

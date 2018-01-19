// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_TRAJECTORY_HPP__
#define __I_TRAJECTORY_HPP__

#include <vector>

#define DEFAULT_TRAJECTORY_DURATION 10.0

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
 * @brief Represents a trajectory.
 */
class ITrajectory
{
public:

    /**
     * @brief Get trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool getDuration(double* duration) const = 0;

    /**
     * @brief Position of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param position A generic position, see subclasses for units.
     *
     * @return true on success, false otherwise
     */
    virtual bool getPosition(const double movementTime, std::vector<double>& position) = 0;

    /**
     * @brief Velocity of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param velocity A generic velocity, see subclasses for units.
     *
     * @return true on success, false otherwise
     */
    virtual bool getVelocity(const double movementTime, std::vector<double>& velocity) = 0;

    /** Destructor */
    virtual ~ITrajectory() {}
};

}  // namespace roboticslab

#endif  // __I_TRAJECTORY_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_TRAJECTORY_HPP__
#define __I_TRAJECTORY_HPP__

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
 * @brief Base class for a generic trajectory.
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

    /**
     * @brief Acceleration of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param acceleration A generic acceleration, see subclasses for units.
     *
     * @return true on success, false otherwise
     */
    virtual bool getAcceleration(const double movementTime, std::vector<double>& acceleration) = 0;

    /**
     * @brief Set trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool setDuration(const double duration) = 0;

    /**
     * @brief Add a waypoint to the trajectory
     *
     * @param waypoint Position information of a generic waypoint, see subclasses for units.
     * @param waypointVelocity Velocity information of a generic waypoint, see subclasses for units.
     * @param waypointAcceleration Acceleration information of a generic waypoint, see subclasses for units.
     *
     * @return true on success, false otherwise
     */
    virtual bool addWaypoint(const std::vector<double>& waypoint,
                             const std::vector<double>& waypointVelocity = std::vector<double>(),
                             const std::vector<double>& waypointAcceleration = std::vector<double>()) = 0;

    virtual bool configurePath(const int pathType) = 0;

    virtual bool configureVelocityProfile(const int velocityProfileType) = 0;

    virtual bool create() = 0;

    virtual bool destroy() = 0;

    /** Destructor */
    virtual ~ITrajectory() {}
};

}  // namespace roboticslab

#endif  // __I_TRAJECTORY_HPP__

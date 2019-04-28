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
    virtual bool getPosition(double movementTime, std::vector<double>& position) = 0;

    /**
     * @brief Velocity of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param velocity A generic velocity, see subclasses for units.
     *
     * @return true on success, false otherwise
     */
    virtual bool getVelocity(double movementTime, std::vector<double>& velocity) = 0;

    /**
     * @brief Acceleration of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param acceleration A generic acceleration, see subclasses for units.
     *
     * @return true on success, false otherwise
     */
    virtual bool getAcceleration(double movementTime, std::vector<double>& acceleration) = 0;

    /**
     * @brief Set trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool setDuration(double duration) = 0;

    /**
     * @brief Set maximum velocity of the trajectory
     *
     * @param maxVelocity The maximum velocity permitted, see subclasses for units.
     *
     * @return true on success, false otherwise
     */
    virtual bool setMaxVelocity(double maxVelocity) = 0;

    /**
     * @brief Set maximum acceleration of the trajectory
     *
     * @param maxAcceleration The maximum acceleration permitted, see subclasses for units.
     *
     * @return true on success, false otherwise
     */
    virtual bool setMaxAcceleration(double maxAcceleration) = 0;

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

    /**
     * @brief Configure the type of path upon creation
     *
     * @param pathType See subclasses for types of paths
     *
     * @return true on success, false otherwise
     */
    virtual bool configurePath(int pathType) = 0;

    /**
     * @brief Configure the type of velocity profile upon creation
     *
     * @param velocityProfileType See subclasses for types of velocity profiles
     *
     * @return true on success, false otherwise
     */
    virtual bool configureVelocityProfile(int velocityProfileType) = 0;

    /** @brief Create the trajectory
     *
     * @return true on success, false otherwise
     */
    virtual bool create() = 0;

    /** @brief Destroy the trajectory
     *
     * @return true on success, false otherwise
     */
    virtual bool destroy() = 0;

    /** Destructor */
    virtual ~ITrajectory() {}
};

}  // namespace roboticslab

#endif  // __I_TRAJECTORY_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_TRAJECTORY_HPP__
#define __I_CARTESIAN_TRAJECTORY_HPP__

#include <vector>

#include "ITrajectory.hpp"

namespace roboticslab
{

/**
 * @ingroup TrajectoryLib
 * @brief Base class for a Cartesian trajectory.
 */
class ICartesianTrajectory : public ITrajectory
{
public:
    //! Lists available Cartesian paths.
    enum cartesian_path
    {
        LINE        ///< A straight line
    };
    //! Lists available Cartesian velocity profiles.
    enum cartesian_velocity_profile
    {
        TRAPEZOIDAL,        ///< A trapezoidal velocity profile
        RECTANGULAR         ///< A rectangular velocity profile
    };

    /**
     * @brief Get trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool getDuration(double* duration) const = 0;

    /**
     * @brief Cartesian position of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param position 6-element vector describing a position in Cartesian space; first
     * three elements denote translation (meters), last three denote rotation in
     * scaled axis-angle representation (radians).
     *
     * @return true on success, false otherwise
     */
    virtual bool getPosition(const double movementTime, std::vector<double>& position) = 0;

    /**
     * @brief Cartesian velocity of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param velocity 6-element vector describing a velocity in Cartesian space; first
     * three elements denote translational velocity (meters/second), last three
     * denote angular velocity (radians/second).
     *
     * @return true on success, false otherwise
     */
    virtual bool getVelocity(const double movementTime, std::vector<double>& velocity) = 0;

    /**
     * @brief Cartesian acceleration of the trajectory at a specific instant in time
     *
     * @param movementTime Time in seconds since start.
     * @param acceleration 6-element vector describing a acceleration in Cartesian space; first
     * three elements denote translational acceleration (meters/second^2), last three
     * denote angular velocity (radians/second^2).
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
     * @param waypoint Position information of a Cartesian waypoint, 6-element vector describing a
     * position in Cartesian space; first three elements denote translation (meters), last three denote
     * rotation in scaled axis-angle representation (radians).
     * @param waypointVelocity Velocity information of a Cartesian waypoint, 6-element vector describing a
     * velocity in Cartesian space; first three elements denote translational velocity (meters/second), last three
     * denote angular velocity (radians/second).
     * @param waypointAcceleration Acceleration information of a Cartesian waypoint, 6-element vector describing a
     * acceleration in Cartesian space; first three elements denote translational acceleration (meters/second^2), last three
     * denote angular acceleration (radians/second^2).
     *
     * @return true on success, false otherwise
     */
    virtual bool addWaypoint(const std::vector<double>& waypoint,
                             const std::vector<double>& waypointVelocity = std::vector<double>(),
                             const std::vector<double>& waypointAcceleration = std::vector<double>()) = 0;

    /**
     * @brief Configure the type of Cartesian path upon creation
     *
     * @param pathType Use a \ref cartesian_path to define the type of Cartesian path
     *
     * @return true on success, false otherwise
     */
    virtual bool configurePath(const int pathType) = 0;

    /**
     * @brief Configure the type of Cartesian velocity profile upon creation
     *
     * @param velocityProfileType Use a \ref cartesian_velocity_profile to define the type of Cartesian velocity profile
     *
     * @return true on success, false otherwise
     */
    virtual bool configureVelocityProfile(const int velocityProfileType) = 0;

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
    virtual ~ICartesianTrajectory() {}
};

}  // namespace roboticslab

#endif  // __I_CARTESIAN_TRAJECTORY_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_TRAJECTORY_HPP__
#define __KDL_TRAJECTORY_HPP__

#include <vector>

#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/path.hpp>
#include <kdl/velocityprofile.hpp>

#include "ICartesianTrajectory.hpp"

#define DURATION_NOT_SET -1

#define DEFAULT_CARTESIAN_MAX_VEL 7.5      // unit/s, enforces a min duration of KDL::Trajectory_Segment
#define DEFAULT_CARTESIAN_MAX_ACC 0.2      // unit/s^2

namespace roboticslab
{

/**
 * @ingroup TrajectoryLib
 * @brief Implements Cartesian trajectory functionalities using KDL.
 */
class KdlTrajectory : public ICartesianTrajectory
{
public:

    /**
     * @brief Constructor
     */
    KdlTrajectory();

    /**
     * @brief Destructor
     */
    virtual ~KdlTrajectory();

    /**
     * @brief Get trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool getDuration(double* duration) const;

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
    virtual bool getPosition(double movementTime, std::vector<double>& position);

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
    virtual bool getVelocity(double movementTime, std::vector<double>& velocity);

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
    virtual bool getAcceleration(double movementTime, std::vector<double>& acceleration);

    /**
     * @brief Set trajectory total duration in seconds
     *
     * @param duration Trajectory total duration in seconds.
     *
     * @return true on success, false otherwise
     */
    virtual bool setDuration(double duration);

    /**
     * @brief Set maximum velocity of the trajectory
     *
     * @param maxVelocity The maximum velocity permitted (meters/second).
     *
     * @return true on success, false otherwise
     */
    virtual bool setMaxVelocity(double maxVelocity);

    /**
     * @brief Set maximum acceleration of the trajectory
     *
     * @param maxAcceleration The maximum acceleration permitted (meters/second^2).
     *
     * @return true on success, false otherwise
     */
    virtual bool setMaxAcceleration(double maxAcceleration);

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
                             const std::vector<double>& waypointAcceleration = std::vector<double>());

    /**
     * @brief Configure the type of Cartesian path upon creation
     *
     * @param pathType Use a \ref cartesian_path to define the type of Cartesian path
     *
     * @return true on success, false otherwise
     */
    virtual bool configurePath(int pathType);

    /**
     * @brief Configure the type of Cartesian velocity profile upon creation
     *
     * @param velocityProfileType Use a \ref cartesian_velocity_profile to define the type of Cartesian velocity profile
     *
     * @return true on success, false otherwise
     */
    virtual bool configureVelocityProfile(int velocityProfileType);

    /** @brief Create the trajectory
     *
     * @return true on success, false otherwise
     */
    virtual bool create();

    /** @brief Destroy the trajectory
     *
     * @return true on success, false otherwise
     */
    virtual bool destroy();

private:

    // disable these per the rule of 3
    KdlTrajectory(const KdlTrajectory &);
    KdlTrajectory operator=(const KdlTrajectory &);

    double duration;
    double maxVelocity, maxAcceleration;

    bool configuredPath, configuredVelocityProfile;
    bool velocityDrivenPath;
    bool created;

    KDL::Trajectory* currentTrajectory;
    KDL::Path* path;
    KDL::VelocityProfile * velocityProfile;

    std::vector<KDL::Frame> frames;
    std::vector<KDL::Twist> twists;
};

}  // namespace roboticslab

#endif  // __KDL_TRAJECTORY_HPP__

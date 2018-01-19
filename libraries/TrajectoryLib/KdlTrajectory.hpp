// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINE_TRAJECTORY_HPP__
#define __LINE_TRAJECTORY_HPP__

#include <string>
#include <vector>

#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>

#include "ICartesianTrajectory.hpp"

namespace roboticslab
{

/**
 * @ingroup TrajectoryLib
 * @brief Implements a line trajectory.
 */
class KdlTrajectory : public ICartesianTrajectory
{
public:

    KdlTrajectory();

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
    virtual bool getPosition(const double movementTime, std::vector<double>& position);

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
    virtual bool getVelocity(const double movementTime, std::vector<double>& velocity);

    bool configurePath(const std::vector<double> &src, const std::vector<double> &dest);
    bool create();
    bool destroy();

private:

    KDL::Trajectory_Segment* currentTrajectory;
    KDL::RotationalInterpolation_SingleAxis* _orient;
    KDL::Path* path;

};

}  // namespace roboticslab

#endif  // __LINE_TRAJECTORY_HPP__

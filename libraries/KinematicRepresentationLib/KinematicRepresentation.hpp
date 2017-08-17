// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KINEMATIC_REPRESENTATION_HPP__
#define __KINEMATIC_REPRESENTATION_HPP__

#include <vector>

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-libraries
 * @defgroup KinematicRepresentationLib
 *
 * @brief Contains classes related to conversion mechanisms
 * between different coordinate and orientation systems.
 */

enum coordinate_system
{
    CARTESIAN,   ///< (x distance, y distance, z distance)
    CYLINDRICAL, ///< (radial distance, azimuthal angle, height)
    SPHERICAL    ///< (radial distance, polar angle, azimuthal angle)
};

enum orientation_system
{
    AXIS_ANGLE,        ///< (axis_x, axis_y, axis_z, rotation angle) [axis as unit vector]
    AXIS_ANGLE_SCALED, ///< (axis_x, axis_y, axis_z) [axis' norm is the rotation angle]
    RPY,               ///< fixed axes, roll is axis_x
    EULER_YZ,          ///< as ::EULER_ZYZ, preceded by rotation about the azimuthal angle got from x-y coordinates
    EULER_ZYZ          ///< mobile axes
};

/**
 * @ingroup KinematicRepresentationLib
 * @brief Stores pose values.
 */
class KinPose
{
public:
    /**
     * @brief Stores the translation and rotation values of a specific pose.
     * @param x Vector describing a three-dimensional pose (translation + rotation).
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true/false on success or failure.
     */
    bool storePose(const std::vector<double> & x, coordinate_system coord, orientation_system orient);

    /**
     * @brief Retrieves the translation and rotation values of a specific pose.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return Vector describing a three-dimensional pose (translation + rotation).
     */
    std::vector<double> retrievePose(coordinate_system coord, orientation_system orient) const;
};

/**
 * @ingroup KinematicRepresentationLib
 * @brief Stores velocity values.
 */
class KinVelocity
{
public:
    /**
     * @brief Stores the translation and rotation values of a specific velocity.
     * @param x Vector describing a three-dimensional velocity (translation + rotation).
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true/false on success or failure.
     */
    bool storeVelocity(const std::vector<double> & xdot, coordinate_system coord, orientation_system orient);

    /**
     * @brief Retrieves the translation and rotation values of a specific velocity.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return Vector describing a three-dimensional velocity (translation + rotation).
     */
    std::vector<double> retrieveVelocity(coordinate_system coord, orientation_system orient) const;
};

/**
 * @ingroup KinematicRepresentationLib
 * @brief Stores acceleration values.
 */
class KinAcceleration
{
public:
    /**
     * @brief Stores the translation and rotation values of a specific acceleration.
     * @param x Vector describing a three-dimensional acceleration (translation + rotation).
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true/false on success or failure.
     */
    bool storeAcceleration(const std::vector<double> & xdotdot, coordinate_system coord, orientation_system orient);

    /**
     * @brief Retrieves the translation and rotation values of a specific acceleration.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return Vector describing a three-dimensional acceleration (translation + rotation).
     */
    std::vector<double> retrieveAcceleration(coordinate_system coord, orientation_system orient) const;
};

}  // namespace roboticslab

#endif  // __KINEMATIC_REPRESENTATION_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KINEMATIC_REPRESENTATION_HPP__
#define __KINEMATIC_REPRESENTATION_HPP__

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
{};

/**
 * @ingroup KinematicRepresentationLib
 * @brief Stores velocity values.
 */
class KinVelocity
{};

/**
 * @ingroup KinematicRepresentationLib
 * @brief Stores acceleration values.
 */
class KinAcceleration
{};

}  // namespace roboticslab

#endif  // __KINEMATIC_REPRESENTATION_HPP__

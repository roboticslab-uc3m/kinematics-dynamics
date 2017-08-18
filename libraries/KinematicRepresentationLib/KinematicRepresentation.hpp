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
 * @brief Contains utilities related to conversion mechanisms
 * between different coordinate and orientation systems.
 */

/**
 * @ingroup KinematicRepresentationLib
 * @brief Collection of static methods to perform geometric
 * transformations.
 */
class KinRepresentation
{
public:
    /**
     * @brief Lists available translational representations.
     */
    enum coordinate_system
    {
        CARTESIAN,   ///< (x distance, y distance, z distance)
        CYLINDRICAL, ///< (radial distance, azimuthal angle, height)
        SPHERICAL    ///< (radial distance, polar angle, azimuthal angle)
    };

    /**
     * @brief Lists available rotational representations.
     */
    enum orientation_system
    {
        AXIS_ANGLE,        ///< (axis_x, axis_y, axis_z, rotation angle) [axis need not to be normalized]
        AXIS_ANGLE_SCALED, ///< (axis_x, axis_y, axis_z) [axis' norm is the rotation angle]
        RPY,               ///< fixed axes, roll is axis_x
        EULER_YZ,          ///< as @ref EULER_ZYZ, preceded by rotation about the azimuthal angle got from x-y coordinates
        EULER_ZYZ          ///< mobile axes
    };

    /**
     * @brief Converts the translation and rotation values of a specific pose to @ref CARTESIAN/@ref AXIS_ANGLE_SCALED systems.
     * @param x_in Input vector describing a three-dimensional pose (translation + rotation).
     * @param x_out Output vector describing the same pose in @ref CARTESIAN/@ref AXIS_ANGLE_SCALED notation.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true on success, false otherwise.
     */
    static bool encodePose(const std::vector<double> &x_in, std::vector<double> &x_out,
            coordinate_system coord, orientation_system orient);

    /**
     * @brief Converts the translation and rotation values of a specific pose to the chosen representation systems.
     * @param x_in Input vector describing a three-dimensional pose in @ref CARTESIAN/@ref AXIS_ANGLE_SCALED systems.
     * @param x_out Output vector describing the same pose in the chosen representation system.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true on success, false otherwise.
     */
    static bool decodePose(const std::vector<double> &x_in, std::vector<double> &x_out,
            coordinate_system coord, orientation_system orient);

    /**
     * @brief Converts the translation and rotation values of a specific velocity to @ref CARTESIAN/@ref AXIS_ANGLE_SCALED systems.
     * @param x_in Input vector describing a three-dimensional velocity (translation + rotation).
     * @param x_out Output vector describing the same velocity in @ref CARTESIAN/@ref AXIS_ANGLE_SCALED systems.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true on success, false otherwise.
     */
    static bool encodeVelocity(const std::vector<double> &xdot_in, std::vector<double> &xdot_out,
            coordinate_system coord, orientation_system orient);

    /**
     * @brief Retrieves the translation and rotation values of a specific velocity to the chosen representation systems.
     * @param x_in Input vector describing a three-dimensional velocity in @ref CARTESIAN/@ref AXIS_ANGLE_SCALED systems.
     * @param x_out Output vector describing the same velocity in the chosen representation system.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true on success, false otherwise.
     */
    static bool decodeVelocity(const std::vector<double> &xdot_in, std::vector<double> &xdot_out,
            coordinate_system coord, orientation_system orient);

    /**
     * @brief Converts the translation and rotation values of a specific acceleration to @ref CARTESIAN/@ref AXIS_ANGLE_SCALED systems.
     * @param x_in Input vector describing a three-dimensional acceleration (translation + rotation).
     * @param x_out Output vector describing the same acceleration in @ref CARTESIAN/@ref AXIS_ANGLE_SCALED systems.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true on success, false otherwise.
     */
    static bool encodeAcceleration(const std::vector<double> &xdotdot_in, std::vector<double> &xdotdot_out,
            coordinate_system coord, orientation_system orient);

    /**
     * @brief Retrieves the translation and rotation values of a specific acceleration to the chosen representation systems.
     * @param x_in Input vector describing a three-dimensional acceleration in @ref CARTESIAN/@ref AXIS_ANGLE_SCALED systems.
     * @param x_out Output vector describing the same acceleration in the chosen representation system.
     * @param coord Coordinate system for the translational part.
     * @param orient Orientation system for the rotational part.
     * @return true on success, false otherwise.
     */
    static bool decodeAcceleration(const std::vector<double> &xdotdot_in, std::vector<double> &xdotdot_out,
            coordinate_system coord, orientation_system orient);

private:
    KinRepresentation() {}

    /**
     * @brief Checks if input vector has the expected size.
     * @param v_in Input vector (translation + rotation).
     * @param orient Orientation system for the rotational part.
     * @param expectedSize Minimum expected size for the input vector in the chosen representation system.
     * @return true if size is adequate, false otherwise.
     */
    static bool checkVectorSize(const std::vector<double> &v_in, orientation_system orient, int *expectedSize);
};

}  // namespace roboticslab

#endif  // __KINEMATIC_REPRESENTATION_HPP__

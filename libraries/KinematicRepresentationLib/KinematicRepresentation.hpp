// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KINEMATIC_REPRESENTATION_HPP__
#define __KINEMATIC_REPRESENTATION_HPP__

#include <string>
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
namespace KinRepresentation
{

//! Lists available translational representations.
enum class coordinate_system
{
    CARTESIAN,   ///< (x distance, y distance, z distance)
    CYLINDRICAL, ///< (radial distance, azimuthal angle, height)
    SPHERICAL,   ///< (radial distance, polar angle, azimuthal angle)
    NONE         ///< omit coordinate system in resulting combined coord+orient representation
};

//! Lists available rotational representations.
enum class orientation_system
{
    AXIS_ANGLE,        ///< (axis_x, axis_y, axis_z, rotation angle) [axis needs not to be normalized]
    AXIS_ANGLE_SCALED, ///< (axis_x, axis_y, axis_z) [axis' norm is the rotation angle]
    RPY,               ///< fixed axes, roll is axis_x
    EULER_YZ,          ///< as @ref EULER_ZYZ, preceded by rotation about the azimuthal angle got from x-y coordinates
    EULER_ZYZ,         ///< mobile axes
    POLAR_AZIMUTH,     ///< (polar angle, azimuthal angle)
    NONE               ///< omit orientation system in resulting combined coord+orient representation
};

//! Lists recognized angular units.
enum class angular_units
{
    DEGREES, ///< degrees (deg)
    RADIANS  ///< radians (rad)
};

/**
 * @brief Converts the translation and rotation values of a specific pose to @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 *
 * Supports in-place transformation.
 *
 * @param x_in Input vector describing a three-dimensional pose (translation + rotation).
 * @param x_out Output vector describing the same pose in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED notation.
 * @param coord Coordinate system for the translational part.
 * @param orient Orientation system for the rotational part.
 * @param angle Units in which angular values are expressed.
 *
 * @return true on success, false otherwise.
 */
bool encodePose(const std::vector<double> & x_in, std::vector<double> & x_out,
        coordinate_system coord, orientation_system orient, angular_units angle = angular_units::RADIANS);

/**
 * @brief Converts the translation and rotation values of a specific pose to the chosen representation systems.
 *
 * Supports in-place transformation.
 *
 * @param x_in Input vector describing a three-dimensional pose in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 * @param x_out Output vector describing the same pose in the chosen representation system.
 * @param coord Coordinate system for the translational part.
 * @param orient Orientation system for the rotational part.
 * @param angle Units in which angular values are expressed.
 *
 * @return true on success, false otherwise.
 */
bool decodePose(const std::vector<double> & x_in, std::vector<double> & x_out,
        coordinate_system coord, orientation_system orient, angular_units angle = angular_units::RADIANS);

/**
 * @brief Converts the translation and rotation values of a specific velocity to @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 *
 * Supports in-place transformation.
 *
 * @param x_in Input vector describing a three-dimensional pose (translation + rotation).
 * @param xdot_in Input vector describing a three-dimensional velocity (translation + rotation).
 * @param xdot_out Output vector describing the same velocity in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 * @param coord Coordinate system for the translational part.
 * @param orient Orientation system for the rotational part.
 * @param angle Units in which angular values are expressed.
 *
 * @return true on success, false otherwise.
 */
bool encodeVelocity(const std::vector<double> & x_in, const std::vector<double> & xdot_in,
        std::vector<double> & xdot_out, coordinate_system coord, orientation_system orient,
        angular_units angle = angular_units::RADIANS);

/**
 * @brief Converts the translation and rotation values of a specific velocity to the chosen representation systems.
 *
 * Supports in-place transformation.
 *
 * @param x_in Input vector describing a three-dimensional pose in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 * @param xdot_in Input vector describing a three-dimensional velocity in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 * @param xdot_out Output vector describing the same velocity in the chosen representation system.
 * @param coord Coordinate system for the translational part.
 * @param orient Orientation system for the rotational part.
 * @param angle Units in which angular values are expressed.
 *
 * @return true on success, false otherwise.
 */
bool decodeVelocity(const std::vector<double> & x_in, const std::vector<double> & xdot_in,
        std::vector<double> & xdot_out, coordinate_system coord, orientation_system orient,
        angular_units angle = angular_units::RADIANS);

/**
 * @brief Converts the translation and rotation values of a specific acceleration to @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 *
 * Supports in-place transformation.
 *
 * @param x_in Input vector describing a three-dimensional pose (translation + rotation).
 * @param xdot_in Input vector describing a three-dimensional velocity (translation + rotation).
 * @param xdotdot_in Input vector describing a three-dimensional acceleration (translation + rotation).
 * @param xdotdot_out Output vector describing the same acceleration in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 * @param coord Coordinate system for the translational part.
 * @param orient Orientation system for the rotational part.
 * @param angle Units in which angular values are expressed.
 *
 * @return true on success, false otherwise.
 */
bool encodeAcceleration(const std::vector<double> & x_in, const std::vector<double> & xdot_in,
        const std::vector<double> & xdotdot_in, std::vector<double> & xdotdot_out,
        coordinate_system coord, orientation_system orient, angular_units angle = angular_units::RADIANS);

/**
 * @brief Converts the translation and rotation values of a specific acceleration to the chosen representation systems.
 *
 * Supports in-place transformation.
 *
 * @param x_in Input vector describing a three-dimensional pose in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 * @param xdot_in Input vector describing a three-dimensional velocity in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 * @param xdotdot_in Input vector describing a three-dimensional acceleration in @ref coordinate_system::CARTESIAN and
 * @ref orientation_system::AXIS_ANGLE_SCALED systems.
 * @param xdotdot_out Output vector describing the same acceleration in the chosen representation system.
 * @param coord Coordinate system for the translational part.
 * @param orient Orientation system for the rotational part.
 * @param angle Units in which angular values are expressed.
 *
 * @return true on success, false otherwise.
 */
bool decodeAcceleration(const std::vector<double> & x_in, const std::vector<double> & xdot_in,
        const std::vector<double> & xdotdot_in, std::vector<double> & xdotdot_out,
        coordinate_system coord, orientation_system orient, angular_units angle = angular_units::RADIANS);

/**
 * @brief Converts degrees to radians.
 *
 * @param deg Angle value expressed in degrees.
 *
 * @return Same value expressed in radians.
 */
double degToRad(double deg);

/**
 * @brief Converts radians to degrees.
 *
 * @param rad Angle value expressed in radians.
 *
 * @return Same value expressed in degrees.
 */
double radToDeg(double rad);

/**
 * @brief Parses input string, returns matching enumerator value
 *
 * Input string    | Enum value
 * --------------- | ----------
 * cartesian       | @ref coordinate_system::CARTESIAN
 * cylindrical     | @ref coordinate_system::CYLINDRICAL
 * spherical       | @ref coordinate_system::SPHERICAL
 * none            | @ref coordinate_system::NONE
 *
 * @param str Input string.
 * @param coord See @ref coordinate_system.
 * @param fallback Default value if no match found.
 *
 * @return true if match found, false otherwise
 */
bool parseEnumerator(const std::string & str, coordinate_system * coord,
        coordinate_system fallback = coordinate_system::CARTESIAN);

/**
 * @brief Parses input string, returns matching enumerator value
 *
 * Input string    | Enum value
 * --------------- | ----------
 * axisAngle       | @ref orientation_system::AXIS_ANGLE
 * axisAngleScaled | @ref orientation_system::AXIS_ANGLE_SCALED
 * RPY             | @ref orientation_system::RPY
 * eulerYZ         | @ref orientation_system::EULER_YZ
 * eulerZYZ        | @ref orientation_system::EULER_ZYZ
 * polarAzimuth    | @ref orientation_system::POLAR_AZIMUTH
 * none            | @ref orientation_system::NONE
 *
 * @param str Input string.
 * @param orient See @ref orientation_system.
 * @param fallback Default value if no match found.
 *
 * @return true if match found, false otherwise
 */
bool parseEnumerator(const std::string & str, orientation_system * orient,
        orientation_system fallback = orientation_system::AXIS_ANGLE_SCALED);

/**
 * @brief Parses input string, returns matching enumerator value
 *
 * Input string    | Enum value
 * --------------- | ----------
 * degrees         | @ref angular_units::DEGREES
 * radians         | @ref angular_units::RADIANS
 *
 * @param str Input string.
 * @param units See @ref angular_units.
 * @param fallback Default value if no match found.
 *
 * @return true if match found, false otherwise
 */
bool parseEnumerator(const std::string & str, angular_units * units,
        angular_units fallback = angular_units::RADIANS);

} // namespace KinRepresentation
} // namespace roboticslab

#endif // __KINEMATIC_REPRESENTATION_HPP__

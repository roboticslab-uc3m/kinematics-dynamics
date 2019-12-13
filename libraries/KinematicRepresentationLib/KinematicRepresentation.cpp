// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinematicRepresentation.hpp"

#include <cmath>

#include <kdl/frames.hpp>
#include <kdl/utilities/utility.h>

#include <ColorDebug.h>

namespace
{
    using namespace roboticslab::KinRepresentation;

    inline double degToRadHelper(angular_units angle, double val)
    {
        return angle == angular_units::RADIANS ? val : degToRad(val);
    }

    inline double radToDegHelper(angular_units angle, double val)
    {
        return angle == angular_units::RADIANS ? val: radToDeg(val);
    }

    bool checkVectorSize(const std::vector<double> & v_in, coordinate_system coord, orientation_system orient, int * expectedSize)
    {
        int accSize = 0;

        switch (coord)
        {
        case coordinate_system::CARTESIAN:
        case coordinate_system::CYLINDRICAL:
        case coordinate_system::SPHERICAL:
            accSize += 3;
            break;
        case coordinate_system::NONE:
            break;
        default:
            return false;
        }

        switch (orient)
        {
        case orientation_system::AXIS_ANGLE:
            accSize += 4;
            break;
        case orientation_system::AXIS_ANGLE_SCALED:
        case orientation_system::RPY:
        case orientation_system::EULER_ZYZ:
            accSize += 3;
            break;
        case orientation_system::EULER_YZ:
        case orientation_system::POLAR_AZIMUTH:
            accSize += 2;
            break;
        case orientation_system::NONE:
            break;
        default:
            return false;
        }

        *expectedSize = accSize;
        return v_in.size() >= accSize;
    }
}

namespace roboticslab
{

namespace KinRepresentation
{

// -----------------------------------------------------------------------------

bool encodePose(const std::vector<double> & x_in, std::vector<double> & x_out,
        coordinate_system coord, orientation_system orient, angular_units angle)
{
    int expectedSize;

    if (!checkVectorSize(x_in, coord, orient, &expectedSize))
    {
        CD_ERROR("Size error; expected: %d, was: %d\n", expectedSize, x_in.size());
        return false;
    }

    // expand current size if needed, but never truncate
    x_out.resize(std::max<int>(6, x_out.size()));
    int off = coord == coordinate_system::NONE ? 0 : 3;

    switch (orient)
    {
    case orientation_system::AXIS_ANGLE:
    {
        KDL::Rotation rot = KDL::Rotation::Rot(KDL::Vector(x_in[0 + off], x_in[1 + off], x_in[2 + off]), degToRadHelper(angle, x_in[3 + off]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case orientation_system::AXIS_ANGLE_SCALED:
    {
        x_out[3] = degToRadHelper(angle, x_in[0 + off]);
        x_out[4] = degToRadHelper(angle, x_in[1 + off]);
        x_out[5] = degToRadHelper(angle, x_in[2 + off]);
        break;
    }
    case orientation_system::RPY:
    {
        KDL::Rotation rot = KDL::Rotation::RPY(degToRadHelper(angle, x_in[0 + off]), degToRadHelper(angle, x_in[1 + off]), degToRadHelper(angle, x_in[2 + off]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case orientation_system::EULER_YZ:
    {
        if (coord == coordinate_system::NONE)
        {
            CD_ERROR("Mandatory coordinate system missing for orientation EULER_YZ.\n");
            return false;
        }

        double alpha = std::atan2(x_in[1], x_in[0]);
        KDL::Rotation rot = KDL::Rotation::EulerZYZ(alpha, degToRadHelper(angle, x_in[3]), degToRadHelper(angle, x_in[4]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case orientation_system::EULER_ZYZ:
    {
        KDL::Rotation rot = KDL::Rotation::EulerZYZ(degToRadHelper(angle, x_in[0 + off]), degToRadHelper(angle, x_in[1 + off]), degToRadHelper(angle, x_in[2 + off]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case orientation_system::POLAR_AZIMUTH:
    {
        KDL::Rotation rot = KDL::Rotation::Rot2(KDL::Vector(-std::sin(degToRadHelper(angle, x_in[1 + off])), std::cos(degToRadHelper(angle, x_in[1 + off])), 0.0), degToRadHelper(angle, x_in[0 + off]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z(); // always 0.0
        break;
    }
    case orientation_system::NONE:
        x_out[3] = x_out[4] = x_out[5] = 0.0;
        break;
    default:
        return false;
    }

    // truncate extra elements
    x_out.resize(6);

    switch (coord)
    {
    case coordinate_system::CARTESIAN:
        x_out[0] = x_in[0];
        x_out[1] = x_in[1];
        x_out[2] = x_in[2];
        break;
    case coordinate_system::CYLINDRICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case coordinate_system::SPHERICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case coordinate_system::NONE:
        x_out[0] = x_out[1] = x_out[2] = 0.0;
        break;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool decodePose(const std::vector<double> & x_in, std::vector<double> & x_out,
        coordinate_system coord, orientation_system orient, angular_units angle)
{
    int expectedSize;

    if (!checkVectorSize(x_in, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE_SCALED, &expectedSize))
    {
        CD_ERROR("Size error; expected: %d, was: %d\n", expectedSize, x_in.size());
        return false;
    }

    int off = coord == coordinate_system::NONE ? 0 : 3;

    switch (orient)
    {
    case orientation_system::AXIS_ANGLE:
    {
        x_out.resize(std::max<int>(4 + off, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        x_out[3 + off] = radToDegHelper(angle, axis.Norm());
        axis.Normalize();
        x_out[0 + off] = axis.x();
        x_out[1 + off] = axis.y();
        x_out[2 + off] = axis.z();
        x_out.resize(4 + off);
        break;
    }
    case orientation_system::AXIS_ANGLE_SCALED:
    {
        x_out.resize(std::max<int>(3 + off, x_out.size()));
        x_out[0 + off] = radToDegHelper(angle, x_in[3]);
        x_out[1 + off] = radToDegHelper(angle, x_in[4]);
        x_out[2 + off] = radToDegHelper(angle, x_in[5]);
        x_out.resize(3 + off);
        break;
    }
    case orientation_system::RPY:
    {
        x_out.resize(std::max<int>(3 + off, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double roll, pitch, yaw;
        rot.GetRPY(roll, pitch, yaw);
        x_out[0 + off] = radToDegHelper(angle, roll);
        x_out[1 + off] = radToDegHelper(angle, pitch);
        x_out[2 + off] = radToDegHelper(angle, yaw);
        x_out.resize(3 + off);
        break;
    }
    case orientation_system::EULER_YZ:
    {
        x_out.resize(std::max<int>(2 + off, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double alpha, beta, gamma;
        rot.GetEulerZYZ(alpha, beta, gamma);
        x_out[0 + off] = radToDegHelper(angle, beta);
        x_out[1 + off] = radToDegHelper(angle, gamma);
        x_out.resize(2 + off);
        break;
    }
    case orientation_system::EULER_ZYZ:
    {
        x_out.resize(std::max<int>(3 + off, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double alpha, beta, gamma;
        rot.GetEulerZYZ(alpha, beta, gamma);
        x_out[0 + off] = radToDegHelper(angle, alpha);
        x_out[1 + off] = radToDegHelper(angle, beta);
        x_out[2 + off] = radToDegHelper(angle, gamma);
        x_out.resize(3 + off);
        break;
    }
    case orientation_system::POLAR_AZIMUTH:
    {
        if (std::abs(x_in[5]) > KDL::epsilon)
        {
            CD_ERROR("Non-null axis Z rotation component: %f.\n", x_in[5]);
            return false;
        }

        x_out.resize(std::max<int>(2 + off, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        x_out[0 + off] = radToDegHelper(angle, axis.Norm());
        x_out[1 + off] = radToDegHelper(angle, std::atan2(-axis.x(), axis.y()));
        x_out.resize(2 + off);
        break;
    }
    case orientation_system::NONE:
        x_out.resize(off);
        break;
    default:
        return false;
    }

    switch (coord)
    {
    case coordinate_system::CARTESIAN:
        x_out[0] = x_in[0];
        x_out[1] = x_in[1];
        x_out[2] = x_in[2];
        break;
    case coordinate_system::CYLINDRICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case coordinate_system::SPHERICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case coordinate_system::NONE:
        break;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool encodeVelocity(const std::vector<double> & x_in, const std::vector<double> & xdot_in,
        std::vector<double> & xdot_out, coordinate_system coord, orientation_system orient, angular_units angle)
{
    int expectedSize;

    if (!checkVectorSize(xdot_in, coord, orient, &expectedSize))
    {
        CD_ERROR("Size error; expected: %d, was: %d\n", expectedSize, xdot_in.size());
        return false;
    }

    // expand current size if needed, but never truncate
    xdot_out.resize(std::max<int>(6, xdot_out.size()));
    int off = coord == coordinate_system::NONE ? 0 : 3;

    switch (orient)
    {
    case orientation_system::AXIS_ANGLE:
    {
        KDL::Rotation rot = KDL::Rotation::Rot(KDL::Vector(xdot_in[0 + off], xdot_in[1 + off], xdot_in[2 + off]), degToRadHelper(angle, xdot_in[3 + off]));
        KDL::Vector axis = rot.GetRot();
        xdot_out[3] = axis.x();
        xdot_out[4] = axis.y();
        xdot_out[5] = axis.z();
        break;
    }
    case orientation_system::AXIS_ANGLE_SCALED:
    {
        xdot_out[3] = degToRadHelper(angle, xdot_in[0 + off]);
        xdot_out[4] = degToRadHelper(angle, xdot_in[1 + off]);
        xdot_out[5] = degToRadHelper(angle, xdot_in[2 + off]);
        break;
    }
    case orientation_system::RPY:
    {
        // [0 -sa ca*cb]
        // [0  ca sa*cb]
        // [1   0   -sb]
        // where a (alpha): z, b (beta): y, g (gamma, unused): x
        // FIXME: really? review this, check which angle corresponds to which coordinate
        KDL::Vector colX = KDL::Rotation::Identity().UnitZ();
        KDL::Rotation rotZ = KDL::Rotation::RotZ(degToRadHelper(angle, x_in[2 + off]));
        KDL::Vector colY = rotZ * KDL::Rotation::Identity().UnitY();
        KDL::Vector colZ = rotZ * KDL::Rotation::RotY(degToRadHelper(angle, x_in[1 + off])) * KDL::Rotation::Identity().UnitX();
        KDL::Rotation rot(colX, colY, colZ);
        KDL::Vector v_in(degToRadHelper(angle, xdot_in[0 + off]), degToRadHelper(angle, xdot_in[1 + off]), degToRadHelper(angle, xdot_in[2 + off]));
        KDL::Vector v_out = rot * v_in;
        xdot_out[3] = v_out.x();
        xdot_out[4] = v_out.y();
        xdot_out[5] = v_out.z();
        break;
    }
    case orientation_system::EULER_YZ:
        CD_ERROR("Not implemented.\n");
        return false;
    case orientation_system::EULER_ZYZ:
        CD_ERROR("Not implemented.\n");
        return false;
    case orientation_system::POLAR_AZIMUTH:
        CD_ERROR("Not implemented.\n");
        return false;
    case orientation_system::NONE:
        xdot_out[3] = xdot_out[4] = xdot_out[5] = 0.0;
        break;
    default:
        return false;
    }

    // truncate extra elements
    xdot_out.resize(6);

    switch (coord)
    {
    case coordinate_system::CARTESIAN:
        xdot_out[0] = xdot_in[0];
        xdot_out[1] = xdot_in[1];
        xdot_out[2] = xdot_in[2];
        break;
    case coordinate_system::CYLINDRICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case coordinate_system::SPHERICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case coordinate_system::NONE:
        xdot_out[0] = xdot_out[1] = xdot_out[2] = 0.0;
        break;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool decodeVelocity(const std::vector<double> & x_in, const std::vector<double> & xdot_in,
        std::vector<double> & xdot_out, coordinate_system coord, orientation_system orient, angular_units angle)
{
    int expectedSize;

    if (!checkVectorSize(xdot_in, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE_SCALED, &expectedSize))
    {
        CD_ERROR("Size error; expected: %d, was: %d\n", expectedSize, xdot_in.size());
        return false;
    }

    int off = coord == coordinate_system::NONE ? 0 : 3;

    switch (orient)
    {
    case orientation_system::AXIS_ANGLE:
    {
        xdot_out.resize(std::max<int>(4 + off, xdot_out.size()));
        KDL::Vector axis(xdot_in[3], xdot_in[4], xdot_in[5]);
        xdot_out[3 + off] = radToDegHelper(angle, axis.Norm());
        axis.Normalize();
        xdot_out[0 + off] = axis.x();
        xdot_out[1 + off] = axis.y();
        xdot_out[2 + off] = axis.z();
        xdot_out.resize(4 + off);
        break;
    }
    case orientation_system::AXIS_ANGLE_SCALED:
    {
        xdot_out.resize(std::max<int>(3 + off, xdot_out.size()));
        xdot_out[0 + off] = radToDegHelper(angle, xdot_in[3]);
        xdot_out[1 + off] = radToDegHelper(angle, xdot_in[4]);
        xdot_out[2 + off] = radToDegHelper(angle, xdot_in[5]);
        xdot_out.resize(3 + off);
        break;
    }
    case orientation_system::RPY:
    {
        // FIXME: see note at 'encodeVelocity'
        xdot_out.resize(std::max<int>(3 + off, xdot_out.size()));
        KDL::Vector colX = KDL::Rotation::Identity().UnitZ();
        KDL::Rotation rotZ = KDL::Rotation::RotZ(x_in[5]);
        KDL::Vector colY = rotZ * KDL::Rotation::Identity().UnitY();
        KDL::Vector colZ = rotZ * KDL::Rotation::RotY(x_in[4]) * KDL::Rotation::Identity().UnitX();
        KDL::Rotation rot(colX, colY, colZ);
        KDL::Vector v_in(xdot_in[3], xdot_in[4], xdot_in[5]);
        KDL::Vector v_out = rot.Inverse() * v_in;
        xdot_out[0 + off] = radToDegHelper(angle, v_out.x());
        xdot_out[1 + off] = radToDegHelper(angle, v_out.y());
        xdot_out[2 + off] = radToDegHelper(angle, v_out.z());
        xdot_out.resize(3 + off);
        break;
    }
    case orientation_system::EULER_YZ:
        CD_ERROR("Not implemented.\n");
        return false;
    case orientation_system::EULER_ZYZ:
        CD_ERROR("Not implemented.\n");
        return false;
    case orientation_system::POLAR_AZIMUTH:
        CD_ERROR("Not implemented.\n");
        return false;
    case orientation_system::NONE:
        xdot_out.resize(off);
        break;
    default:
        return false;
    }

    switch (coord)
    {
    case coordinate_system::CARTESIAN:
        xdot_out[0] = xdot_in[0];
        xdot_out[1] = xdot_in[1];
        xdot_out[2] = xdot_in[2];
        break;
    case coordinate_system::CYLINDRICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case coordinate_system::SPHERICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case coordinate_system::NONE:
        break;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool encodeAcceleration(const std::vector<double> & x_in, const std::vector<double> & xdot_in,
        const std::vector<double> & xdotdot_in, std::vector<double> & xdotdot_out,
        coordinate_system coord, orientation_system orient, angular_units angle)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool decodeAcceleration(const std::vector<double> & x_in, const std::vector<double> & xdot_in,
        const std::vector<double> & xdotdot_in, std::vector<double> & xdotdot_out,
        coordinate_system coord, orientation_system orient, angular_units angle)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

double degToRad(double deg)
{
    return deg * KDL::deg2rad;
}

// -----------------------------------------------------------------------------

double radToDeg(double rad)
{
    return rad * KDL::rad2deg;
}

// -----------------------------------------------------------------------------

bool parseEnumerator(const std::string & str, coordinate_system * coord, coordinate_system fallback)
{
    if (str == "cartesian")
    {
        *coord = coordinate_system::CARTESIAN;
    }
    else if (str == "cylindrical")
    {
        *coord = coordinate_system::CYLINDRICAL;
    }
    else if (str == "spherical")
    {
        *coord = coordinate_system::SPHERICAL;
    }
    else if (str == "none")
    {
        *coord = coordinate_system::NONE;
    }
    else
    {
        *coord = fallback;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool parseEnumerator(const std::string & str, orientation_system * orient, orientation_system fallback)
{
    if (str == "axisAngle")
    {
        *orient = orientation_system::AXIS_ANGLE;
    }
    else if (str == "axisAngleScaled")
    {
        *orient = orientation_system::AXIS_ANGLE_SCALED;
    }
    else if (str == "RPY")
    {
        *orient = orientation_system::RPY;
    }
    else if (str == "eulerYZ")
    {
        *orient = orientation_system::EULER_YZ;
    }
    else if (str == "eulerZYZ")
    {
        *orient = orientation_system::EULER_ZYZ;
    }
    else if (str == "polarAzimuth")
    {
        *orient = orientation_system::POLAR_AZIMUTH;
    }
    else if (str == "none")
    {
        *orient = orientation_system::NONE;
    }
    else
    {
        *orient = fallback;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool parseEnumerator(const std::string & str, angular_units * units, angular_units fallback)
{
    if (str == "degrees")
    {
        *units = angular_units::DEGREES;
    }
    else if (str == "radians")
    {
        *units = angular_units::RADIANS;
    }
    else
    {
        *units = fallback;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

} // namespace KinRepresentation
} // namespace roboticslab

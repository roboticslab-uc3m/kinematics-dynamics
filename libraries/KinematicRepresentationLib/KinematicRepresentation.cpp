// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinematicRepresentation.hpp"

#include <cmath>

#include <kdl/frames.hpp>
#include <kdl/utilities/utility.h>

#include <ColorDebug.h>

namespace roboticslab
{

namespace
{
    inline double degToRadHelper(KinRepresentation::angular_units angle, double val)
    {
        return angle == KinRepresentation::RADIANS ? val : KinRepresentation::degToRad(val);
    }

    inline double radToDegHelper(KinRepresentation::angular_units angle, double val)
    {
        return angle == KinRepresentation::RADIANS ? val: KinRepresentation::radToDeg(val);
    }
}

// -----------------------------------------------------------------------------

bool KinRepresentation::encodePose(const std::vector<double> &x_in, std::vector<double> &x_out,
        coordinate_system coord, orientation_system orient, angular_units angle)
{
    int expectedSize;

    if (!checkVectorSize(x_in, orient, &expectedSize))
    {
        CD_ERROR("Size error; expected: %d, was: %d\n", expectedSize, x_in.size());
        return false;
    }

    // expand current size if needed, but never truncate
    x_out.resize(std::max<int>(6, x_out.size()));

    switch (orient)
    {
    case AXIS_ANGLE:
    {
        KDL::Rotation rot = KDL::Rotation::Rot(KDL::Vector(x_in[3], x_in[4], x_in[5]), degToRadHelper(angle, x_in[6]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case AXIS_ANGLE_SCALED:
    {
        x_out[3] = degToRadHelper(angle, x_in[3]);
        x_out[4] = degToRadHelper(angle, x_in[4]);
        x_out[5] = degToRadHelper(angle, x_in[5]);
        break;
    }
    case RPY:
    {
        KDL::Rotation rot = KDL::Rotation::RPY(degToRadHelper(angle, x_in[3]), degToRadHelper(angle, x_in[4]), degToRadHelper(angle, x_in[5]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case EULER_YZ:
    {
        double alpha = std::atan2(x_in[1], x_in[0]);
        KDL::Rotation rot = KDL::Rotation::EulerZYZ(alpha, degToRadHelper(angle, x_in[3]), degToRadHelper(angle, x_in[4]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case EULER_ZYZ:
    {
        KDL::Rotation rot = KDL::Rotation::EulerZYZ(degToRadHelper(angle, x_in[3]), degToRadHelper(angle, x_in[4]), degToRadHelper(angle, x_in[5]));
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    default:
        return false;
    }

    // truncate extra elements
    x_out.resize(6);

    switch (coord)
    {
    case CARTESIAN:
        x_out[0] = x_in[0];
        x_out[1] = x_in[1];
        x_out[2] = x_in[2];
        break;
    case CYLINDRICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case SPHERICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::decodePose(const std::vector<double> &x_in, std::vector<double> &x_out,
        coordinate_system coord, orientation_system orient, angular_units angle)
{
    int expectedSize;

    if (!checkVectorSize(x_in, AXIS_ANGLE_SCALED, &expectedSize))
    {
        CD_ERROR("Size error; expected: %d, was: %d\n", expectedSize, x_in.size());
        return false;
    }

    switch (orient)
    {
    case AXIS_ANGLE:
    {
        x_out.resize(std::max<int>(7, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        x_out[6] = radToDegHelper(angle, axis.Norm());
        axis.Normalize();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        x_out.resize(7);
        break;
    }
    case AXIS_ANGLE_SCALED:
    {
        x_out.resize(std::max<int>(6, x_out.size()));
        x_out[3] = radToDegHelper(angle, x_in[3]);
        x_out[4] = radToDegHelper(angle, x_in[4]);
        x_out[5] = radToDegHelper(angle, x_in[5]);
        x_out.resize(6);
        break;
    }
    case RPY:
    {
        x_out.resize(std::max<int>(6, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double roll, pitch, yaw;
        rot.GetRPY(roll, pitch, yaw);
        x_out[3] = radToDegHelper(angle, roll);
        x_out[4] = radToDegHelper(angle, pitch);
        x_out[5] = radToDegHelper(angle, yaw);
        x_out.resize(6);
        break;
    }
    case EULER_YZ:
    {
        x_out.resize(std::max<int>(5, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double alpha, beta, gamma;
        rot.GetEulerZYZ(alpha, beta, gamma);
        x_out[3] = radToDegHelper(angle, beta);
        x_out[4] = radToDegHelper(angle, gamma);
        x_out.resize(5);
        break;
    }
    case EULER_ZYZ:
    {
        x_out.resize(std::max<int>(6, x_out.size()));
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double alpha, beta, gamma;
        rot.GetEulerZYZ(alpha, beta, gamma);
        x_out[3] = radToDegHelper(angle, alpha);
        x_out[4] = radToDegHelper(angle, beta);
        x_out[5] = radToDegHelper(angle, gamma);
        x_out.resize(6);
        break;
    }
    default:
        return false;
    }

    switch (coord)
    {
    case CARTESIAN:
        x_out[0] = x_in[0];
        x_out[1] = x_in[1];
        x_out[2] = x_in[2];
        break;
    case CYLINDRICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case SPHERICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::encodeVelocity(const std::vector<double> &x_in, const std::vector<double> &xdot_in,
        std::vector<double> &xdot_out, coordinate_system coord, orientation_system orient, angular_units angle)
{
    int expectedSize;

    if (!checkVectorSize(xdot_in, orient, &expectedSize))
    {
        CD_ERROR("Size error; expected: %d, was: %d\n", expectedSize, xdot_in.size());
        return false;
    }

    // expand current size if needed, but never truncate
    xdot_out.resize(std::max<int>(6, xdot_out.size()));

    switch (orient)
    {
    case AXIS_ANGLE:
    {
        KDL::Rotation rot = KDL::Rotation::Rot(KDL::Vector(xdot_in[3], xdot_in[4], xdot_in[5]), degToRadHelper(angle, xdot_in[6]));
        KDL::Vector axis = rot.GetRot();
        xdot_out[3] = axis.x();
        xdot_out[4] = axis.y();
        xdot_out[5] = axis.z();
        break;
    }
    case AXIS_ANGLE_SCALED:
    {
        xdot_out[3] = degToRadHelper(angle, xdot_in[3]);
        xdot_out[4] = degToRadHelper(angle, xdot_in[4]);
        xdot_out[5] = degToRadHelper(angle, xdot_in[5]);
        break;
    }
    case RPY:
    {
        // [0 -sa ca*cb]
        // [0  ca sa*cb]
        // [1   0   -sb]
        // where a (alpha): z, b (beta): y, g (gamma, unused): x
        // FIXME: really? review this, check which angle corresponds to which coordinate
        KDL::Vector colX = KDL::Rotation::Identity().UnitZ();
        KDL::Rotation rotZ = KDL::Rotation::RotZ(degToRadHelper(angle, x_in[5]));
        KDL::Vector colY = rotZ * KDL::Rotation::Identity().UnitY();
        KDL::Vector colZ = rotZ * KDL::Rotation::RotY(degToRadHelper(angle, x_in[4])) * KDL::Rotation::Identity().UnitX();
        KDL::Rotation rot(colX, colY, colZ);
        KDL::Vector v_in(degToRadHelper(angle, xdot_in[3]), degToRadHelper(angle, xdot_in[4]), degToRadHelper(angle, xdot_in[5]));
        KDL::Vector v_out = rot * v_in;
        xdot_out[3] = v_out.x();
        xdot_out[4] = v_out.y();
        xdot_out[5] = v_out.z();
        break;
    }
    case EULER_YZ:
    {
        break;
    }
    case EULER_ZYZ:
    {
        break;
    }
    default:
        return false;
    }

    // truncate extra elements
    xdot_out.resize(6);

    switch (coord)
    {
    case CARTESIAN:
        xdot_out[0] = xdot_in[0];
        xdot_out[1] = xdot_in[1];
        xdot_out[2] = xdot_in[2];
        break;
    case CYLINDRICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case SPHERICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::decodeVelocity(const std::vector<double> &x_in, const std::vector<double> &xdot_in,
        std::vector<double> &xdot_out, coordinate_system coord, orientation_system orient, angular_units angle)
{
    int expectedSize;

    if (!checkVectorSize(xdot_in, AXIS_ANGLE_SCALED, &expectedSize))
    {
        CD_ERROR("Size error; expected: %d, was: %d\n", expectedSize, xdot_in.size());
        return false;
    }

    switch (orient)
    {
    case AXIS_ANGLE:
    {
        xdot_out.resize(std::max<int>(7, xdot_out.size()));
        KDL::Vector axis(xdot_in[3], xdot_in[4], xdot_in[5]);
        xdot_out[6] = radToDegHelper(angle, axis.Norm());
        axis.Normalize();
        xdot_out[3] = axis.x();
        xdot_out[4] = axis.y();
        xdot_out[5] = axis.z();
        xdot_out.resize(7);
        break;
    }
    case AXIS_ANGLE_SCALED:
    {
        xdot_out.resize(std::max<int>(6, xdot_out.size()));
        xdot_out[3] = radToDegHelper(angle, xdot_in[3]);
        xdot_out[4] = radToDegHelper(angle, xdot_in[4]);
        xdot_out[5] = radToDegHelper(angle, xdot_in[5]);
        xdot_out.resize(6);
        break;
    }
    case RPY:
    {
        // FIXME: see note at 'encodeVelocity'
        xdot_out.resize(std::max<int>(6, xdot_out.size()));
        KDL::Vector colX = KDL::Rotation::Identity().UnitZ();
        KDL::Rotation rotZ = KDL::Rotation::RotZ(x_in[5]);
        KDL::Vector colY = rotZ * KDL::Rotation::Identity().UnitY();
        KDL::Vector colZ = rotZ * KDL::Rotation::RotY(x_in[4]) * KDL::Rotation::Identity().UnitX();
        KDL::Rotation rot(colX, colY, colZ);
        KDL::Vector v_in(xdot_in[3], xdot_in[4], xdot_in[5]);
        KDL::Vector v_out = rot.Inverse() * v_in;
        xdot_out[3] = radToDegHelper(angle, v_out.x());
        xdot_out[4] = radToDegHelper(angle, v_out.y());
        xdot_out[5] = radToDegHelper(angle, v_out.z());
        xdot_out.resize(6);
        break;
    }
    case EULER_YZ:
    {
        xdot_out.resize(std::max<int>(5, xdot_out.size()));
        xdot_out.resize(5);
        break;
    }
    case EULER_ZYZ:
    {
        xdot_out.resize(std::max<int>(6, xdot_out.size()));
        xdot_out.resize(6);
        break;
    }
    default:
        return false;
    }

    switch (coord)
    {
    case CARTESIAN:
        xdot_out[0] = xdot_in[0];
        xdot_out[1] = xdot_in[1];
        xdot_out[2] = xdot_in[2];
        break;
    case CYLINDRICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    case SPHERICAL:
        CD_ERROR("Not implemented.\n");
        return false;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::encodeAcceleration(const std::vector<double> &x_in, const std::vector<double> &xdot_in,
        const std::vector<double> &xdotdot_in, std::vector<double> &xdotdot_out,
        coordinate_system coord, orientation_system orient, angular_units angle)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::decodeAcceleration(const std::vector<double> &x_in, const std::vector<double> &xdot_in,
        const std::vector<double> &xdotdot_in, std::vector<double> &xdotdot_out,
        coordinate_system coord, orientation_system orient, angular_units angle)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

double KinRepresentation::degToRad(double deg)
{
    return deg * KDL::deg2rad;
}

// -----------------------------------------------------------------------------

double KinRepresentation::radToDeg(double rad)
{
    return rad * KDL::rad2deg;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::parseEnumerator(const std::string &str, orientation_system *orient, orientation_system fallback)
{
    if (str == "axisAngle")
    {
        *orient = AXIS_ANGLE;
    }
    else if (str == "axisAngleScaled")
    {
        *orient = AXIS_ANGLE_SCALED;
    }
    else if (str == "RPY")
    {
        *orient = RPY;
    }
    else if (str == "eulerYZ")
    {
        *orient = EULER_YZ;
    }
    else if (str == "eulerZYZ")
    {
        *orient = EULER_ZYZ;
    }
    else
    {
        *orient = fallback;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::checkVectorSize(const std::vector<double> &v_in, orientation_system orient, int *expectedSize)
{
    switch (orient)
    {
    case AXIS_ANGLE:
        *expectedSize = 7;
        return v_in.size() >= *expectedSize;
    case AXIS_ANGLE_SCALED:
        *expectedSize = 6;
        return v_in.size() >= *expectedSize;
    case RPY:
        *expectedSize = 6;
        return v_in.size() >= *expectedSize;
    case EULER_YZ:
        *expectedSize = 5;
        return v_in.size() >= *expectedSize;
    case EULER_ZYZ:
        *expectedSize = 6;
        return v_in.size() >= *expectedSize;
    default:
        *expectedSize = 0;
        return false;
    }

    return true;
}

}  // namespace roboticslab

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinematicRepresentation.hpp"

#include <cmath>

#include <kdl/frames.hpp>

#include <ColorDebug.hpp>

namespace roboticslab
{

// -----------------------------------------------------------------------------

bool KinRepresentation::encodePose(const std::vector<double> &x_in, std::vector<double> &x_out,
        coordinate_system coord, orientation_system orient)
{
    switch (orient)
    {
    case AXIS_ANGLE:
    {
        x_out.resize(7);
        KDL::Rotation rot = KDL::Rotation::Rot(KDL::Vector(x_in[3], x_in[4], x_in[5]), x_in[6]);
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case AXIS_ANGLE_SCALED:
    {
        x_out.resize(6);
        x_out[3] = x_in[3];
        x_out[4] = x_in[4];
        x_out[5] = x_in[5];
        break;
    }
    case RPY:
    {
        x_out.resize(6);
        KDL::Rotation rot = KDL::Rotation::RPY(x_in[3], x_in[4], x_in[5]);
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case EULER_YZ:
    {
        x_out.resize(5);
        double alpha = std::atan2(x_in[1], x_in[0]);
        KDL::Rotation rot = KDL::Rotation::EulerZYZ(alpha, x_in[4], x_in[5]);
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case EULER_ZYZ:
    {
        x_out.resize(6);
        KDL::Rotation rot = KDL::Rotation::EulerZYZ(x_in[3], x_in[4], x_in[5]);
        KDL::Vector axis = rot.GetRot();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
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

bool KinRepresentation::decodePose(const std::vector<double> &x_in, std::vector<double> &x_out,
        coordinate_system coord, orientation_system orient)
{
    switch (orient)
    {
    case AXIS_ANGLE:
    {
        x_out.resize(7);
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        x_out[6] = axis.Norm();
        axis.Normalize();
        x_out[3] = axis.x();
        x_out[4] = axis.y();
        x_out[5] = axis.z();
        break;
    }
    case AXIS_ANGLE_SCALED:
    {
        x_out.resize(6);
        x_out[3] = x_in[3];
        x_out[4] = x_in[4];
        x_out[5] = x_in[5];
        break;
    }
    case RPY:
    {
        x_out.resize(6);
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double roll, pitch, yaw;
        rot.GetRPY(roll, pitch, yaw);
        x_out[3] = roll;
        x_out[4] = pitch;
        x_out[5] = yaw;
        break;
    }
    case EULER_YZ:
    {
        x_out.resize(5);
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double alpha, beta, gamma;
        rot.GetEulerZYZ(alpha, beta, gamma);
        x_out[3] = beta;
        x_out[4] = gamma;
        break;
    }
    case EULER_ZYZ:
    {
        x_out.resize(6);
        KDL::Vector axis(x_in[3], x_in[4], x_in[5]);
        KDL::Rotation rot = KDL::Rotation::Rot(axis, axis.Norm());
        double alpha, beta, gamma;
        rot.GetEulerZYZ(alpha, beta, gamma);
        x_out[3] = alpha;
        x_out[4] = beta;
        x_out[5] = gamma;
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

bool KinRepresentation::encodeVelocity(const std::vector<double> &xdot_in, std::vector<double> &xdot_out,
        coordinate_system coord, orientation_system orient)
{
    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::decodeVelocity(const std::vector<double> &xdot_in, std::vector<double> &xdot_out,
        coordinate_system coord, orientation_system orient)
{
    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::encodeAcceleration(const std::vector<double> &xdotdot_in, std::vector<double> &xdotdot_out,
        coordinate_system coord, orientation_system orient)
{
    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::decodeAcceleration(const std::vector<double> &xdotdot_in, std::vector<double> &xdotdot_out,
        coordinate_system coord, orientation_system orient)
{
    return true;
}

}  // namespace roboticslab

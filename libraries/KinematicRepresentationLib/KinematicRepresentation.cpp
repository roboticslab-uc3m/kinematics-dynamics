// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinematicRepresentation.hpp"

#include <ColorDebug.hpp>

namespace roboticslab
{

// -----------------------------------------------------------------------------

bool KinRepresentation::encodePose(const std::vector<double> &x_in, std::vector<double> &x_out,
        coordinate_system coord, orientation_system orient)
{
    return true;
}

// -----------------------------------------------------------------------------

bool KinRepresentation::decodePose(const std::vector<double> &x_in, std::vector<double> &x_out,
        coordinate_system coord, orientation_system orient)
{
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

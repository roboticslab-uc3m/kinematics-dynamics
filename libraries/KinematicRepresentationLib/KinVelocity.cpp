// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinematicRepresentation.hpp"

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::KinVelocity::storeVelocity(const std::vector<double> & xdot, coordinate_system coord, orientation_system orient)
{
    return true;
}

// -----------------------------------------------------------------------------

std::vector<double> roboticslab::KinVelocity::retrieveVelocity(coordinate_system coord, orientation_system orient) const
{
    return std::vector<double>();
}

// -----------------------------------------------------------------------------

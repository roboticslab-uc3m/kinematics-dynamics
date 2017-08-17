// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KinematicRepresentation.hpp"

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::KinAcceleration::storeAcceleration(const std::vector<double> & xdotdot, coordinate_system coord, orientation_system orient)
{
    return true;
}

// -----------------------------------------------------------------------------

std::vector<double> roboticslab::KinAcceleration::retrieveAcceleration(coordinate_system coord, orientation_system orient) const
{
    return std::vector<double>();
}

// -----------------------------------------------------------------------------

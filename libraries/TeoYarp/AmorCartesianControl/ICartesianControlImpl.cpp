// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::AmorCartesianControl::stat(int &state, std::vector<double> &x)
{
    AMOR_VECTOR7 positions;

    if (amor_get_cartesian_position(handle, positions) != AMOR_SUCCESS)
    {
        CD_ERROR("Could not retrieve current cartesian position.\n");
        return false;
    }

    x.resize(6);

    x[0] = positions[0] * 0.001;  // [m]
    x[1] = positions[1] * 0.001;
    x[2] = positions[2] * 0.001;

    x[3] = toDeg(positions[3]);  // [deg]
    x[4] = toDeg(positions[4]);
    x[5] = toDeg(positions[5]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movj(const std::vector<double> &xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::relj(const std::vector<double> &xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movl(const std::vector<double> &xd)
{
    AMOR_VECTOR7 positions;

    positions[0] = xd[0] * 1000;  // [mm]
    positions[1] = xd[1] * 1000;
    positions[2] = xd[2] * 1000;

    positions[3] = toRad(xd[3]);  // [rad]
    positions[4] = toRad(xd[4]);
    positions[5] = toRad(xd[5]);

    return amor_set_cartesian_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movv(const std::vector<double> &xdotd)
{
    AMOR_VECTOR7 velocities;

    velocities[0] = xdotd[0] * 1000;  // [mm/s]
    velocities[1] = xdotd[1] * 1000;
    velocities[2] = xdotd[2] * 1000;

    velocities[3] = toRad(xdotd[3]);  // [rad/s]
    velocities[4] = toRad(xdotd[4]);
    velocities[5] = toRad(xdotd[5]);

    return amor_set_cartesian_velocities(handle, velocities) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::gcmp()
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::forc(const std::vector<double> &td)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::stopControl()
{
    return amor_controlled_stop(handle) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::fwd(const std::vector<double> &rot)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::bkwd(const std::vector<double> &rot)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::rot(const std::vector<double> &rot)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::vmos(const std::vector<double> &xdot)
{
    AMOR_VECTOR7 velocities;

    velocities[0] = xdot[0] * 1000;  // [mm/s]
    velocities[1] = xdot[1] * 1000;
    velocities[2] = xdot[2] * 1000;

    velocities[3] = toRad(xdot[3]);  // [rad/s]
    velocities[4] = toRad(xdot[4]);
    velocities[5] = toRad(xdot[5]);

    return amor_set_cartesian_velocities(handle, velocities) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::pose(const std::vector<double> &x)
{
    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <cmath>

#include "KinematicRepresentation.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::AmorCartesianControl::stat(int &state, std::vector<double> &x)
{
    AMOR_VECTOR7 positions;

    if (amor_get_cartesian_position(handle, positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    x.resize(6);

    x[0] = positions[0] * 0.001;  // [m]
    x[1] = positions[1] * 0.001;
    x[2] = positions[2] * 0.001;

    x[3] = positions[3];  // [rad]
    x[4] = positions[4];
    x[5] = positions[5];

    KinRepresentation::encodePose(x, x, KinRepresentation::CARTESIAN, KinRepresentation::RPY);

    state = 0;  // dummy value

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS);

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    if (!iCartesianSolver->invKin(xd, currentQ, q))
    {
        CD_ERROR("invKin failed.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movj(const std::vector<double> &xd)
{
    std::vector<double> qd;

    if (!inv(xd, qd))
    {
        CD_ERROR("inv failed.\n");
        return false;
    }

    AMOR_VECTOR7 positions;

    for (int i = 0; i < qd.size(); i++)
    {
        positions[i] = KinRepresentation::degToRad(qd[i]);
    }

    if (amor_set_positions(handle, positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::relj(const std::vector<double> &xd)
{
    int state;
    std::vector<double> x;

    if (!stat(state, x))
    {
        CD_ERROR("stat failed.\n");
        return false;
    }

    for (int i = 0; i < xd.size(); i++)
    {
        x[i] += xd[i];
    }

    return movj(x);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movl(const std::vector<double> &xd)
{
    std::vector<double> xd_rpy;

    KinRepresentation::decodePose(xd, xd_rpy, KinRepresentation::CARTESIAN, KinRepresentation::RPY);

    AMOR_VECTOR7 positions;

    positions[0] = xd_rpy[0] * 1000;  // [mm]
    positions[1] = xd_rpy[1] * 1000;
    positions[2] = xd_rpy[2] * 1000;

    positions[3] = xd_rpy[3];  // [rad]
    positions[4] = xd_rpy[4];
    positions[5] = xd_rpy[5];

    if (amor_set_cartesian_positions(handle, positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movv(const std::vector<double> &xdotd)
{
    int state;
    std::vector<double> xCurrent;

    if (!stat(state, xCurrent))
    {
        CD_ERROR("stat failed\n");
        return false;
    }

    std::vector<double> xdotd_rpy;

    KinRepresentation::decodeVelocity(xCurrent, xdotd, xdotd_rpy, KinRepresentation::CARTESIAN, KinRepresentation::RPY);

    AMOR_VECTOR7 velocities;

    velocities[0] = xdotd_rpy[0] * 1000;  // [mm/s]
    velocities[1] = xdotd_rpy[1] * 1000;
    velocities[2] = xdotd_rpy[2] * 1000;

    // FIXME: un-shuffle coordinates
    velocities[3] = xdotd_rpy[4];  // [rad/s]
    velocities[4] = -xdotd_rpy[5];
    velocities[5] = xdotd_rpy[3];

    if (amor_set_cartesian_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
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
    if (amor_controlled_stop(handle) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::tool(const std::vector<double> &x)
{
    CD_WARNING("Tool change is not supported on AMOR.\n");
    return false;
}

// -----------------------------------------------------------------------------

void roboticslab::AmorCartesianControl::fwd(const std::vector<double> &rot, double step)
{
    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS), qdot;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    std::vector<double> xdotee(6);
    xdotee[2] = std::max(step, 0.0);
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if (!iCartesianSolver->diffInvKinEE(currentQ, xdotee, qdot))
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    AMOR_VECTOR7 velocities;

    for (int i = 0; i < qdot.size(); i++)
    {
        velocities[i] = KinRepresentation::degToRad(qdot[i]);
    }

    if (amor_set_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::AmorCartesianControl::bkwd(const std::vector<double> &rot, double step)
{
    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS), qdot;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    std::vector<double> xdotee(6);
    xdotee[2] = -std::max(step, 0.0);
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if (!iCartesianSolver->diffInvKinEE(currentQ, xdotee, qdot))
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    AMOR_VECTOR7 velocities;

    for (int i = 0; i < qdot.size(); i++)
    {
        velocities[i] = KinRepresentation::degToRad(qdot[i]);
    }

    if (amor_set_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::AmorCartesianControl::rot(const std::vector<double> &rot)
{
    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS), qdot;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    std::vector<double> xdotee(6);
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if (!iCartesianSolver->diffInvKinEE(currentQ, xdotee, qdot))
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    AMOR_VECTOR7 velocities;

    for (int i = 0; i < qdot.size(); i++)
    {
        velocities[i] = KinRepresentation::degToRad(qdot[i]);
    }

    if (amor_set_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::AmorCartesianControl::pan(const std::vector<double> &transl)
{
    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS), qdot;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    std::vector<double> xdotee(6);
    xdotee[0] = transl[0];
    xdotee[1] = transl[1];
    xdotee[2] = transl[2];

    if (!iCartesianSolver->diffInvKinEE(currentQ, xdotee, qdot))
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    AMOR_VECTOR7 velocities;

    for (int i = 0; i < qdot.size(); i++)
    {
        velocities[i] = KinRepresentation::degToRad(qdot[i]);
    }

    if (amor_set_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::AmorCartesianControl::vmos(const std::vector<double> &xdot)
{
    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS), qdot;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot))
    {
        CD_ERROR("diffInvKin failed.\n");
        return;
    }

    AMOR_VECTOR7 velocities;

    for (int i = 0; i < qdot.size(); i++)
    {
        velocities[i] = KinRepresentation::degToRad(qdot[i]);
    }

    if (amor_set_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::AmorCartesianControl::eff(const std::vector<double> &xdotee)
{
    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS), qdot;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    if (!iCartesianSolver->diffInvKinEE(currentQ, xdotee, qdot))
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    AMOR_VECTOR7 velocities;

    for (int i = 0; i < qdot.size(); i++)
    {
        velocities[i] = KinRepresentation::degToRad(qdot[i]);
    }

    if (amor_set_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::AmorCartesianControl::pose(const std::vector<double> &x, double interval)
{
    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS);

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    std::vector<double> xd;

    if (!iCartesianSolver->fwdKinError(x, currentQ, xd))
    {
        CD_ERROR("fwdKinError failed.\n");
        return;
    }

    std::vector<double> xdot(xd.size());
    const double factor = 0.05 / interval;  // DEFAULT_GAIN = 0.05

    for (int i = 0; i < xd.size(); i++)
    {
        xdot[i] = xd[i] * factor;
    }

    std::vector<double> qdot;

    if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot))
    {
        CD_ERROR("diffInvKin failed.\n");
        return;
    }

    AMOR_VECTOR7 velocities;

    for (int i = 0; i < qdot.size(); i++)
    {
        velocities[i] = KinRepresentation::degToRad(qdot[i]);
    }

    if (amor_set_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return;
    }
}

// -----------------------------------------------------------------------------

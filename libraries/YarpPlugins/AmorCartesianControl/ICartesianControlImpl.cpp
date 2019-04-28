// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

#include "KinematicRepresentation.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::AmorCartesianControl::stat(std::vector<double> &x, int * state, double * timestamp)
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

    if (state != 0)
    {
        *state = currentState;
    }

    if (timestamp != 0)
    {
        *timestamp = yarp::os::Time::now();
    }

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

    if (!iCartesianSolver->invKin(xd, currentQ, q, referenceFrame))
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

    currentState = VOCAB_CC_MOVJ_CONTROLLING;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::relj(const std::vector<double> &xd)
{
    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        return movj(xd);
    }

    std::vector<double> x;

    if (!stat(x))
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
    std::vector<double> xd_obj;

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
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

        std::vector<double> x_base_tcp;

        if (!iCartesianSolver->fwdKin(currentQ, x_base_tcp))
        {
            CD_ERROR("fwdKin failed.\n");
            return false;
        }

        if (!iCartesianSolver->changeOrigin(xd, x_base_tcp, xd_obj))
        {
            CD_ERROR("changeOrigin failed.\n");
            return false;
        }
    }
    else
    {
        xd_obj = xd;
    }

    std::vector<double> xd_rpy;

    KinRepresentation::decodePose(xd_obj, xd_rpy, KinRepresentation::CARTESIAN, KinRepresentation::RPY);

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

    currentState = VOCAB_CC_MOVL_CONTROLLING;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movv(const std::vector<double> &xdotd)
{
    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        CD_WARNING("TCP frame not supported yet in movv command.\n");
        return false;
    }

    std::vector<double> xCurrent;

    if (!stat(xCurrent))
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

    currentState = VOCAB_CC_MOVV_CONTROLLING;

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
    currentState = VOCAB_CC_NOT_CONTROLLING;

    if (amor_controlled_stop(handle) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::wait(double timeout)
{
    if (currentState != VOCAB_CC_MOVJ_CONTROLLING && currentState != VOCAB_CC_MOVL_CONTROLLING)
    {
        return true;
    }

    AMOR_RESULT res = AMOR_SUCCESS;
    amor_movement_status status;

    double start = yarp::os::Time::now();

    do
    {
        if (timeout != 0.0 && yarp::os::Time::now() - start > timeout)
        {
            CD_WARNING("Timeout reached (%f seconds), stopping control.\n", timeout);
            stopControl();
            break;
        }

        res = amor_get_movement_status(handle, &status);

        if (res == AMOR_FAILED)
        {
            CD_ERROR("%s\n", amor_error());
            break;
        }

        yarp::os::Time::delay(waitPeriodMs / 1000.0);
    }
    while (status != AMOR_MOVEMENT_STATUS_FINISHED);

    currentState = VOCAB_CC_NOT_CONTROLLING;

    return res == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::tool(const std::vector<double> &x)
{
    CD_WARNING("Tool change is not supported on AMOR.\n");
    return false;
}

// -----------------------------------------------------------------------------

void roboticslab::AmorCartesianControl::twist(const std::vector<double> &xdot)
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

    if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot, referenceFrame))
    {
        CD_ERROR("diffInvKin failed.\n");
        return;
    }

    if (!checkJointVelocities(qdot))
    {
        amor_controlled_stop(handle);
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

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->fwdKin(currentQ, x_base_tcp))
    {
        CD_ERROR("fwdKin failed.\n");
        return;
    }

    std::vector<double> x_obj;

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        if (!iCartesianSolver->changeOrigin(x, x_base_tcp, x_obj))
        {
            CD_ERROR("changeOrigin failed.\n");
            return;
        }
    }
    else
    {
        x_obj = x;
    }

    std::vector<double> xd;

    if (!iCartesianSolver->poseDiff(x_obj, x_base_tcp, xd))
    {
        CD_ERROR("fwdKinError failed.\n");
        return;
    }

    std::vector<double> xdot(xd.size());
    const double factor = gain / interval;

    for (int i = 0; i < xd.size(); i++)
    {
        xdot[i] = xd[i] * factor;
    }

    std::vector<double> qdot;

    if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot, referenceFrame))
    {
        CD_ERROR("diffInvKin failed.\n");
        return;
    }

    if (!checkJointVelocities(qdot))
    {
        amor_controlled_stop(handle);
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

void roboticslab::AmorCartesianControl::movi(const std::vector<double> &x)
{
    CD_WARNING("movi not supported, falling back to movj.\n");
    movj(x);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::setParameter(int vocab, double value)
{
    if (currentState != VOCAB_CC_NOT_CONTROLLING)
    {
        CD_ERROR("Unable to set config parameter while controlling.\n");
        return false;
    }

    switch (vocab)
    {
    case VOCAB_CC_CONFIG_GAIN:
        if (value < 0.0)
        {
            CD_ERROR("Controller gain cannot be negative.\n");
            return false;
        }
        gain = value;
        break;
    case VOCAB_CC_CONFIG_MAX_JOINT_VEL:
        if (value <= 0.0)
        {
            CD_ERROR("Maximum joint velocity cannot be negative nor zero.\n");
            return false;
        }
        maxJointVelocity = value;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        if (value <= 0.0)
        {
            CD_ERROR("Wait period cannot be negative nor zero.\n");
            return false;
        }
        waitPeriodMs = value;
        break;
    case VOCAB_CC_CONFIG_FRAME:
        if (value != ICartesianSolver::BASE_FRAME && value != ICartesianSolver::TCP_FRAME)
        {
            CD_ERROR("Unrecognized or unsupported reference frame vocab.\n");
            return false;
        }
        referenceFrame = static_cast<ICartesianSolver::reference_frame>(value);
        break;
    default:
        CD_ERROR("Unrecognized or unsupported config parameter key: %s.\n", yarp::os::Vocab::decode(vocab).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::getParameter(int vocab, double * value)
{
    switch (vocab)
    {
    case VOCAB_CC_CONFIG_GAIN:
        *value = gain;
        break;
    case VOCAB_CC_CONFIG_MAX_JOINT_VEL:
        *value = maxJointVelocity;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        *value = waitPeriodMs;
        break;
    case VOCAB_CC_CONFIG_FRAME:
        *value = referenceFrame;
        break;
    default:
        CD_ERROR("Unrecognized or unsupported config parameter key: %s.\n", yarp::os::Vocab::decode(vocab).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::setParameters(const std::map<int, double> & params)
{
    if (currentState != VOCAB_CC_NOT_CONTROLLING)
    {
        CD_ERROR("Unable to set config parameters while controlling.\n");
        return false;
    }

    bool ok = true;

    for (std::map<int, double>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
        ok &= setParameter(it->first, it->second);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::getParameters(std::map<int, double> & params)
{
    params.insert(std::make_pair(VOCAB_CC_CONFIG_GAIN, gain));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_MAX_JOINT_VEL, maxJointVelocity));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_WAIT_PERIOD, waitPeriodMs));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_FRAME, referenceFrame));
    return true;
}

// -----------------------------------------------------------------------------

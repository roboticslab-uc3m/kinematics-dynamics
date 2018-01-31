// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <cmath>

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::checkJointVelocities(const std::vector<double> &qdot)
{
    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if (std::abs(qdot[i]) > maxJointVelocity)
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], maxJointVelocity);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::performDiffInvKin(const std::vector<double> & currentQ,
                                                          const std::vector<double> & xdot,
                                                          std::vector<double> & qdot)
{
    if (referenceFrame == BASE_FRAME)
    {
        if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot))
        {
            CD_ERROR("diffInvKin failed.\n");
            return false;
        }
    }
    else if (referenceFrame == TCP_FRAME)
    {
        if (!iCartesianSolver->diffInvKinEE(currentQ, xdot, qdot))
        {
            CD_ERROR("diffInvKinEE failed.\n");
            return false;
        }
    }
    else
    {
        CD_ERROR("Unsupported reference frame.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::performDiffInvKin(const std::vector<double> & currentQ,
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

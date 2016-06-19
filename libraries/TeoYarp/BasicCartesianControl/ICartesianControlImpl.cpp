// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::BasicCartesianControl::stat(std::vector<double> &x)
{
    std::vector<double> q(numRobotJoints);
    if ( ! iEncoders->getEncoders( q.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->fwdKin(q,x) )
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

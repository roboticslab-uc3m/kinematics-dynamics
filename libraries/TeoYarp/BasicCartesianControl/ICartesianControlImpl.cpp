// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool teo::BasicCartesianControl::stat(int &state, std::vector<double> &x)
{
    std::vector<double> qCurrent(numRobotJoints);
    if ( ! iEncoders->getEncoders( qCurrent.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->fwdKin(qCurrent,x) )
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }
    state = currentState;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    std::vector<double> qCurrent(numRobotJoints);
    if ( ! iEncoders->getEncoders( qCurrent.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->invKin(xd,qCurrent,q) )
    {
        CD_ERROR("invKin failed.\n");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::movj(std::vector<double> &xd)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::movl(std::vector<double> &xd)
{
    currentState = VOCAB_CC_MOVEL_CONTROLLING;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::stop()
{
    return true;
}

// -----------------------------------------------------------------------------

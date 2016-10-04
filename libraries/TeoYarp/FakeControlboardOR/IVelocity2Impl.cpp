// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboardOR.hpp"

// ------------------ IVelocity2 Related ----------------------------------------

bool teo::FakeControlboardOR::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("\n");
    // must implement mask!
    return velocityMove(spds);
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefVelocity(const int joint, double *vel)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefVelocities(double *vels)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setVelPid(int j, const yarp::dev::Pid &pid)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setVelPids(const yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getVelPid(int j, yarp::dev::Pid *pid)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getVelPids(yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboardOR.hpp"

// ------------------- IPositionControl2 Related --------------------------------

bool teo::FakeControlboardOR::positionMove(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("\n");
    // must implement mask!
    return positionMove(refs);
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::stop(const int n_joint, const int *joints)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getTargetPosition(const int joint, double *ref)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getTargetPositions(double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboardOR::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

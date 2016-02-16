// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ------------------- IPositionControl2 Related --------------------------------

bool teo::FakeControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    // must implement mask!
    return positionMove(refs);
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::stop(const int n_joint, const int *joints)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTargetPosition(const int joint, double *ref)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTargetPositions(double *refs)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    return true;
}

// -----------------------------------------------------------------------------

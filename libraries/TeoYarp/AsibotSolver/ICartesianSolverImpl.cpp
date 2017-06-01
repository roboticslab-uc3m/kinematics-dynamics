// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getNumLinks(int* numLinks)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::appendLink(const std::vector<double> &x)
{
    return true;
}

// --------------------------------------------------------------------------

bool roboticslab::AsibotSolver::restoreOriginalChain()
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x)
{
    std::vector<double> qInRad(q);

    for (std::vector<double>::iterator it = qInRad.begin(); it != qInRad.end(); it++)
    {
        *it = toRad(*it);
    }

    // P = prime
    double prP = A1 * std::sin(qInRad[1]) + A2 * std::sin(qInRad[1] + qInRad[2]) + A3 * std::sin(qInRad[1] + qInRad[2] + qInRad[3]);
    double phP = A1 * std::cos(qInRad[1]) + A2 * std::cos(qInRad[1] + qInRad[2]) + A3 * std::cos(qInRad[1] + qInRad[2] + qInRad[3]);
    double oyP = q[1] + q[2] + q[3];  // [deg]

    x.resize(NUM_MOTORS);
    x[0] = prP * std::cos(qInRad[0]);
    x[1] = prP * std::sin(qInRad[0]);
    x[2] = phP + A0;  // pz = pzP
    x[3] = oyP;  // = pitchP
    x[4] = q[4];  // = ozPP

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::fwdKinError(const std::vector<double> &xd, const std::vector<double> &q, std::vector<double> &x)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invDyn(const std::vector<double> &q,std::vector<double> &t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setLimits(const std::vector<double> &qMin, const std::vector<double> &qMax)
{
    return true;
}

// -----------------------------------------------------------------------------

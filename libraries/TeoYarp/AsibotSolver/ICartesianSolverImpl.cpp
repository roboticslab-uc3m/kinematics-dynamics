// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getNumJoints(int* numJoints)
{
    *numJoints = NUM_MOTORS;
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

    for (std::vector<double>::iterator it = qInRad.begin(); it != qInRad.end(); ++it)
    {
        *it = toRad(*it);
    }

    // P = prime
    double prP = A1 * std::sin(qInRad[1]) + A2 * std::sin(qInRad[1] + qInRad[2]) + A3 * std::sin(qInRad[1] + qInRad[2] + qInRad[3]);
    double phP = A1 * std::cos(qInRad[1]) + A2 * std::cos(qInRad[1] + qInRad[2]) + A3 * std::cos(qInRad[1] + qInRad[2] + qInRad[3]);
    double oyP = q[1] + q[2] + q[3];  // [deg]

    x.resize(5);  // translation[3] + rotation[2]

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
    std::vector<double> currentX;
    fwdKin(q, currentX);

    x.resize(5);

    x[0] = xd[0] - currentX[0];
    x[1] = xd[1] - currentX[1];
    x[2] = xd[2] - currentX[2];
    x[3] = xd[3] - currentX[3];
    x[4] = xd[4] - currentX[4];

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q)
{
    double ozdRad = std::atan2(xd[1], xd[0]);

    double prPd = std::sqrt(xd[0] * xd[0] + xd[1] * xd[1]);
    double phPd = xd[2] - A0;
    double oyPd = xd[3];

    double prWd = prPd - A3 * std::sin(toRad(oyPd));
    double phWd = phPd - A3 * std::cos(toRad(oyPd));

    double ct2 = (prWd * prWd + phWd * phWd - A1 * A1 - A2 * A2) / (2 * A1 * A2);
    double st2 = std::sqrt(1 - ct2 * ct2);  // forces elbow-up in ASIBOT
    //double st2 = -std::sqrt(1 - ct2 * ct2);  // forces elbow-down in ASIBOT

    double t2Rad = std::atan2(st2, ct2);

    double st1 = ((A1 + A2 * ct2) * prWd - A2 * st2 * phWd) / (phWd * phWd + prWd * prWd);
    double ct1 = ((A1 + A2 * ct2) * phWd + A2 * st2 * prWd) / (phWd * phWd + prWd * prWd);
    // double ct1 = (phWd + A2 * st1 * st2) / (A1 + A2 * ct2);  // alternative method for same result

    double t1Rad = std::atan2(st1, ct1);

    q.resize(NUM_MOTORS);

    q[0] = toDeg(ozdRad);
    q[1] = toDeg(t1Rad);
    q[2] = toDeg(t2Rad);
    q[3] = oyPd - q[1] - q[2];
    q[4] = xd[4];  // ozPP

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

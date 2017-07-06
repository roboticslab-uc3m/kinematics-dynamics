// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

#include <yarp/math/FrameTransform.h>

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

    double s1 = std::sin(qInRad[0]);
    double c1 = std::cos(qInRad[0]);
    double s2 = std::sin(qInRad[1]);
    double c2 = std::cos(qInRad[1]);

    double s23 = std::sin(qInRad[1] + qInRad[2]);
    double c23 = std::cos(qInRad[1] + qInRad[2]);

    double s234 = std::sin(qInRad[1] + qInRad[2] + qInRad[3]);
    double c234 = std::cos(qInRad[1] + qInRad[2] + qInRad[3]);

    // P = prime
    double prP = A1 * s2 + A2 * s23 + A3 * s234;
    double phP = A1 * c2 + A2 * c23 + A3 * c234;
    double oyP = q[1] + q[2] + q[3];  // [deg]

    x.resize(5);  // translation[3] + rotation[2]

    x[0] = prP * c1;
    x[1] = prP * s1;
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

    x.resize(6);

    x[0] = xd[0] - currentX[0];
    x[1] = xd[1] - currentX[1];
    x[2] = xd[2] - currentX[2];

    yarp::sig::Vector eulerDesired(3);
    yarp::sig::Vector eulerCurrent(3);

    eulerDesired[0] = std::atan2(xd[1], xd[0]);
    eulerDesired[1] = toRad(xd[3]);
    eulerDesired[2] = toRad(xd[4]);

    eulerCurrent[0] = std::atan2(currentX[1], currentX[0]);
    eulerCurrent[1] = toRad(currentX[3]);
    eulerCurrent[2] = toRad(currentX[4]);

    yarp::sig::Matrix Hdesired = yarp::math::euler2dcm(eulerDesired);
    yarp::sig::Matrix Hcurrent = yarp::math::euler2dcm(eulerCurrent);

    yarp::math::FrameTransform frameDesired, frameCurrent;

    frameDesired.fromMatrix(Hdesired);
    frameCurrent.fromMatrix(Hcurrent);

    yarp::sig::Vector rpyDesired = frameDesired.getRPYRot();
    yarp::sig::Vector rpyCurrent = frameCurrent.getRPYRot();

    x[3] = toDeg(rpyDesired[0] - rpyCurrent[0]);
    x[4] = toDeg(rpyDesired[1] - rpyCurrent[1]);
    x[5] = toDeg(rpyDesired[2] - rpyCurrent[2]);

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
    std::vector<double> qInRad(q);

    for (std::vector<double>::iterator it = qInRad.begin(); it != qInRad.end(); ++it)
    {
        *it = toRad(*it);
    }

    double s1 = std::sin(qInRad[0]);
    double c1 = std::cos(qInRad[0]);
    double s2 = std::sin(qInRad[1]);
    double c2 = std::cos(qInRad[1]);
    double s5 = std::sin(qInRad[4]);
    double c5 = std::cos(qInRad[4]);

    double s23 = std::sin(qInRad[1] + qInRad[2]);
    double c23 = std::cos(qInRad[1] + qInRad[2]);

    double s234 = std::sin(qInRad[1] + qInRad[2] + qInRad[3]);
    double c234 = std::cos(qInRad[1] + qInRad[2] + qInRad[3]);

    double den = 1 - s234 * s234 * c5 * c5;
    double sqrtden = std::sqrt(den);

    yarp::sig::Matrix Ja(6, 5);

    Ja(0, 0) = -s1 * (A3 * s234 + A2 * s23 + A1 * s2);
    Ja(0, 1) =  c1 * (A3 * c234 + A2 * c23 + A1 * c2);
    Ja(0, 2) =  c1 * (A3 * c234 + A2 * c23);
    Ja(0, 3) =  c1 * A3 * c234;
    Ja(0, 4) =  0;

    Ja(1, 0) = c1 * (A3 * s234 + A2 * s23 + A1 * s2);
    Ja(1, 1) = s1 * (A3 * c234 + A2 * c23 + A1 * c2);
    Ja(1, 2) = s1 * (A3 * c234 + A2 * c23);
    Ja(1, 3) = s1 * A3 * c234;
    Ja(1, 4) = 0;

    Ja(2, 0) = 0;
    Ja(2, 1) = -A3 * s234 - A2 * s23 - A1 * s2;
    Ja(2, 2) = -A3 * s234 - A2 * s23;
    Ja(2, 3) = -A3 * s234;
    Ja(2, 4) = 0;

    Ja(3, 0) = 0;
    Ja(3, 1) = s5 / den;
    Ja(3, 2) = Ja(3, 1);
    Ja(3, 3) = Ja(3, 1);
    Ja(3, 4) = (s234 * c234 * c5) / den;

    Ja(4, 0) = 0;
    Ja(4, 1) = (c234 * c5) / sqrtden;
    Ja(4, 2) = Ja(4, 1);
    Ja(4, 3) = Ja(4, 1);
    Ja(4, 4) = -(s234 * s5) / sqrtden;

    Ja(5, 0) = 1;
    Ja(5, 1) = (s234 * s5 * c5) / den;
    Ja(5, 2) = Ja(5, 1);
    Ja(5, 3) = Ja(5, 1);
    Ja(5, 4) = c234 / den;

    yarp::sig::Matrix Ja_inv = yarp::math::pinv(Ja, 1.0e-2);

    yarp::sig::Vector xdotv(6);

    xdotv[0] = xdot[0];
    xdotv[1] = xdot[1];
    xdotv[2] = xdot[2];
    xdotv[3] = toRad(xdot[3]);
    xdotv[4] = toRad(xdot[4]);
    xdotv[5] = toRad(xdot[5]);

    using namespace yarp::math;
    yarp::sig::Vector qdotv = Ja_inv * xdotv;

    qdot.resize(NUM_MOTORS);

    qdot[0] = toDeg(qdotv[0]);
    qdot[1] = toDeg(qdotv[1]);
    qdot[2] = toDeg(qdotv[2]);
    qdot[3] = toDeg(qdotv[3]);
    qdot[4] = toDeg(qdotv[4]);

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

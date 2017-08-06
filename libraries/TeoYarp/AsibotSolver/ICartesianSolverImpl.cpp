// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

#include <cmath>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/sig/Matrix.h>

#include "YarpTinyMath.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getNumJoints(int* numJoints)
{
    *numJoints = NUM_MOTORS;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::appendLink(const std::vector<double> &x)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// --------------------------------------------------------------------------

bool roboticslab::AsibotSolver::restoreOriginalChain()
{
    CD_WARNING("Not implemented.\n");
    return false;
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

    yarp::sig::Matrix rotDesired = yarp::math::euler2dcm(eulerDesired).submatrix(0, 2, 0, 2);
    yarp::sig::Matrix rotCurrent = yarp::math::euler2dcm(eulerCurrent).submatrix(0, 2, 0, 2);

    using namespace yarp::math;
    yarp::sig::Matrix rotCurrentToDesired = rotCurrent.transposed() * rotDesired;
    yarp::sig::Vector axisAngle = yarp::math::dcm2axis(rotCurrentToDesired);
    yarp::sig::Vector axis = axisAngle.subVector(0, 2) * axisAngle[3];
    yarp::sig::Vector rotd = rotCurrent * axis;

    x[3] = rotd[0];
    x[4] = rotd[1];
    x[5] = rotd[2];

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q)
{
    double ozdRad = std::atan2(xd[1], xd[0]);

    double prPd = std::sqrt(xd[0] * xd[0] + xd[1] * xd[1]);
    double phPd = xd[2] - A0;
    double oyPd = xd[3];

    if (std::sqrt(prPd * prPd + phPd * phPd) > A1 + A2 + A3)
    {
        CD_ERROR("Target out of reach.\n");
        return false;
    }

    double prWd = prPd - A3 * std::sin(toRad(oyPd));
    double phWd = phPd - A3 * std::cos(toRad(oyPd));

    double len_2 = phWd * phWd + prWd * prWd;

    double ct2 = (len_2 - A1 * A1 - A2 * A2) / (2 * A1 * A2);
    double st2 = std::sqrt(1 - ct2 * ct2);  // forces elbow-up in ASIBOT
    //double st2 = -std::sqrt(1 - ct2 * ct2);  // forces elbow-down in ASIBOT

    double t2Rad = std::atan2(st2, ct2);
    //double t2Rad = std::atan2(-st2, ct2);  // or just '-std::atan(st2, ct2)' (elbow-down)

    // 'u': elbow-up; 'd': elbow-down
    double st1u = ((A1 + A2 * ct2) * prWd - A2 * st2 * phWd) / len_2;
    double ct1u = ((A1 + A2 * ct2) * phWd + A2 * st2 * prWd) / len_2;
    // double ct1u = (phWd + A2 * st1 * st2) / (A1 + A2 * ct2);  // alternative method for same result

    double st1d = ((A1 + A2 * ct2) * prWd + A2 * st2 * phWd) / len_2;
    double ct1d = ((A1 + A2 * ct2) * phWd - A2 * st2 * prWd) / len_2;

    double t1uRad = std::atan2(st1u, ct1u);
    double t1dRad = std::atan2(st1d, ct1d);

    double t3uRad = toRad(oyPd) - t1uRad - t2Rad;
    double t3dRad = toRad(oyPd) - t1dRad + t2Rad;

    if (!conf->configure(toDeg(ozdRad), toDeg(t1uRad), toDeg(t1dRad), toDeg(t2Rad), toDeg(t3uRad), toDeg(t3dRad), xd[4]))
    {
        CD_ERROR("Unable to find a valid configuration within joint limits.\n");
        return false;
    }

    if (!conf->findOptimalConfiguration(qGuess))
    {
        CD_ERROR("findOptimalConfiguration() failed.\n");
        return false;
    }

    conf->retrieveAngles(q);

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
    Ja(3, 1) = -s1;
    Ja(3, 2) = -s1;
    Ja(3, 3) = -s1;
    Ja(3, 4) = c1 * s234;

    Ja(4, 0) = 0;
    Ja(4, 1) = c1;
    Ja(4, 2) = c1;
    Ja(4, 3) = c1;
    Ja(4, 4) = s1 * s234;

    Ja(5, 0) = 1;
    Ja(5, 1) = 0;
    Ja(5, 2) = 0;
    Ja(5, 3) = 0;
    Ja(5, 4) = c234;

    yarp::sig::Matrix Ja_inv = yarp::math::pinv(Ja, 1.0e-2);

    yarp::sig::Vector xdotv(6);

    xdotv[0] = xdot[0];
    xdotv[1] = xdot[1];
    xdotv[2] = xdot[2];
    xdotv[3] = xdot[3];
    xdotv[4] = xdot[4];
    xdotv[5] = xdot[5];

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
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setLimits(const std::vector<double> &qMin, const std::vector<double> &qMax)
{
    for (int motor = 0; motor < NUM_MOTORS; motor++)
    {
        if (qMin[motor] > qMax[motor])
        {
            CD_ERROR("qMin > qMax at joint q%d (%f vs %f).\n", motor + 1, qMin[motor], qMax[motor]);
            return false;
        }
        else if (qMin[motor] == qMax[motor])
        {
            CD_WARNING("qMin = qMax at joint q%d.\n", motor + 1);
        }

        this->qMax[motor] = qMax[motor];
        this->qMin[motor] = qMin[motor];
    }

    return true;
}

// -----------------------------------------------------------------------------

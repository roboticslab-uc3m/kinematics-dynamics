// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkSubproblems.hpp"

#include <cmath>

#include "ScrewTheoryTools.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

PadenKahanOne::PadenKahanOne(int _id, const MatrixExponential & _exp, const KDL::Vector & _p)
    : id(_id),
      exp(_exp),
      p(_p),
      axisPow(vectorPow2(exp.getAxis()))
{}

// -----------------------------------------------------------------------------

bool PadenKahanOne::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const
{
    solutions.resize(PadenKahanOne::solutions());
    JointIdsToSolutions jointIdsToSolutions(1);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * p;

    KDL::Vector u = f - exp.getOrigin();
    KDL::Vector v = k - exp.getOrigin();

    KDL::Vector u_w = axisPow * u;
    KDL::Vector v_w = axisPow * v;

    KDL::Vector u_p = u - u_w;
    KDL::Vector v_p = v - v_w;

    double theta = std::atan2(KDL::dot(exp.getAxis(), u_p * v_p), KDL::dot(u_p, v_p));

    jointIdsToSolutions[0] = std::make_pair(id, normalizeAngle(theta));
    solutions[0] = jointIdsToSolutions;

    return KDL::Equal(u_w, v_w) && KDL::Equal(u_p.Norm(), v_p.Norm());
}

// -----------------------------------------------------------------------------

PadenKahanTwo::PadenKahanTwo(int _id1, int _id2, const MatrixExponential & _exp1, const MatrixExponential & _exp2, const KDL::Vector & _p, const KDL::Vector & _r)
  : id1(_id1),
    id2(_id2),
    exp1(_exp1),
    exp2(_exp2),
    p(_p),
    r(_r),
    axesCross(exp1.getAxis() * exp2.getAxis()),
    axisPow1(vectorPow2(exp1.getAxis())),
    axisPow2(vectorPow2(exp2.getAxis())),
    axesDot(KDL::dot(exp1.getAxis(), exp2.getAxis()))
{}

// -----------------------------------------------------------------------------

bool PadenKahanTwo::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const
{
    solutions.resize(PadenKahanTwo::solutions());
    JointIdsToSolutions jointIdsToSolution1(2), jointIdsToSolution2(2);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * p;

    KDL::Vector u = f - r;
    KDL::Vector v = k - r;

    KDL::Vector u_p = u - axisPow2 * u;
    KDL::Vector v_p = v - axisPow1 * v;

    double axis1dot = KDL::dot(exp1.getAxis(), v);
    double axis2dot = KDL::dot(exp2.getAxis(), u);
    double den = std::pow(axesDot, 2) - 1;

    double alpha = (axesDot * axis2dot - axis1dot) / den;
    double beta = (axesDot * axis1dot - axis2dot) / den;

    KDL::Vector term1 = r + alpha * exp1.getAxis() + beta * exp2.getAxis();

    double gamma2 = (std::pow(u.Norm(), 2) - std::pow(alpha, 2) - std::pow(beta, 2) - 2 * alpha * beta * axesDot) /
            std::pow(axesCross.Norm(), 2);

    bool gamma2_zero = KDL::Equal(gamma2, 0.0);

    jointIdsToSolution1[0].first = jointIdsToSolution2[0].first = id1;
    jointIdsToSolution1[1].first = jointIdsToSolution2[1].first = id2;

    bool ret;

    if (!gamma2_zero && gamma2 > 0.0)
    {
        double gamma = std::sqrt(gamma2);
        KDL::Vector term2 = gamma * axesCross;

        KDL::Vector d = term1 + term2;
        KDL::Vector c = term1 - term2;

        KDL::Vector m = c - r;
        KDL::Vector n = d - r;

        KDL::Vector m1_p = m - axisPow1 * m;
        KDL::Vector m2_p = m - axisPow2 * m;

        KDL::Vector n1_p = n - axisPow1 * n;
        KDL::Vector n2_p = n - axisPow2 * n;

        double theta1_1 = std::atan2(KDL::dot(exp1.getAxis(), m1_p * v_p), KDL::dot(m1_p, v_p));
        double theta2_1 = std::atan2(KDL::dot(exp2.getAxis(), u_p * m2_p), KDL::dot(u_p, m2_p));

        double theta1_2 = std::atan2(KDL::dot(exp1.getAxis(), n1_p * v_p), KDL::dot(n1_p, v_p));
        double theta2_2 = std::atan2(KDL::dot(exp2.getAxis(), u_p * n2_p), KDL::dot(u_p, n2_p));

        jointIdsToSolution1[0].second = normalizeAngle(theta1_1);
        jointIdsToSolution1[1].second = normalizeAngle(theta2_1);

        jointIdsToSolution2[0].second = normalizeAngle(theta1_2);
        jointIdsToSolution2[1].second = normalizeAngle(theta2_2);

        ret = KDL::Equal(m1_p.Norm(), v_p.Norm());
    }
    else
    {
        KDL::Vector n = term1 - r;
        KDL::Vector n1_p = n - axisPow1 * n;
        KDL::Vector n2_p = n - axisPow2 * n;

        double theta1 = std::atan2(KDL::dot(exp1.getAxis(), n1_p * v_p), KDL::dot(n1_p, v_p));
        double theta2 = std::atan2(KDL::dot(exp2.getAxis(), u_p * n2_p), KDL::dot(u_p, n2_p));

        jointIdsToSolution1[0].second = jointIdsToSolution2[0].second = normalizeAngle(theta1);
        jointIdsToSolution1[1].second = jointIdsToSolution2[1].second = normalizeAngle(theta2);

        ret = gamma2_zero && KDL::Equal(n1_p.Norm(), v_p.Norm());
    }

    solutions[0] = jointIdsToSolution1;
    solutions[1] = jointIdsToSolution2;

    return ret;
}

// -----------------------------------------------------------------------------

PadenKahanThree::PadenKahanThree(int _id, const MatrixExponential & _exp, const KDL::Vector & _p, const KDL::Vector & _k)
    : id(_id),
      exp(_exp),
      p(_p),
      k(_k),
      axisPow(vectorPow2(exp.getAxis()))
{}

// -----------------------------------------------------------------------------

bool PadenKahanThree::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const
{
    solutions.resize(PadenKahanThree::solutions());
    JointIdsToSolutions jointIdsToSolution1(1), jointIdsToSolution2(1);

    KDL::Vector f = pointTransform * p;
    KDL::Vector rhsAsVector = rhs * p - k;
    double delta = rhsAsVector.Norm();

    KDL::Vector u = f - exp.getOrigin();
    KDL::Vector v = k - exp.getOrigin();

    KDL::Vector u_p = u - axisPow * u;
    KDL::Vector v_p = v - axisPow * v;

    double alpha = std::atan2(KDL::dot(exp.getAxis(), u_p * v_p), KDL::dot(u_p, v_p));
    double delta_p_2 = std::pow(delta, 2) - std::pow(KDL::dot(exp.getAxis(), f - k), 2);

    double u_p_norm = u_p.Norm();
    double v_p_norm = v_p.Norm();

    double betaCos = (std::pow(u_p_norm, 2) + std::pow(v_p_norm, 2) - delta_p_2) / (2 * u_p_norm * v_p_norm);
    double betaCosAbs = std::abs(betaCos);
    bool beta_zero = KDL::Equal(betaCosAbs, 1.0);

    jointIdsToSolution1[0].first = jointIdsToSolution2[0].first = id;

    bool ret;

    if (!beta_zero && betaCosAbs < 1.0)
    {
        double betaCosCapped = std::max(-1.0, std::min(1.0, betaCos));
        double beta = std::acos(betaCosCapped);

        double theta1 = alpha + beta;
        double theta2 = alpha - beta;

        jointIdsToSolution1[0].second = normalizeAngle(theta1);
        jointIdsToSolution2[0].second = normalizeAngle(theta2);

        ret = true;
    }
    else
    {
        jointIdsToSolution1[0].second = jointIdsToSolution2[0].second = normalizeAngle(alpha);
        ret = beta_zero;
    }

    solutions[0] = jointIdsToSolution1;
    solutions[1] = jointIdsToSolution2;

    return ret;
}

// -----------------------------------------------------------------------------

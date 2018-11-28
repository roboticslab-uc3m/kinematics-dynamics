// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkSubproblems.hpp"

#include <cmath>

#include "ScrewTheoryTools.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

PadenKahanOne::PadenKahanOne(int _id, const MatrixExponential & _exp, const KDL::Vector & _p)
    : id(_id),
      exp(_exp),
      p(_p)
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem::SolutionsVector PadenKahanOne::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform)
{
    SolutionsVector solutions(1);
    JointIdsToSolutionsVector jointIdsToSolutions(1);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * f;

    KDL::Vector u = f - exp.getOrigin();
    KDL::Vector v = k - exp.getOrigin();

    KDL::Rotation axisPow = vectorPow2(exp.getAxis());

    KDL::Vector u_p = u - axisPow * u;
    KDL::Vector v_p = v - axisPow * v;

    double theta = std::atan2(KDL::dot(exp.getAxis(), u_p * v_p), KDL::dot(u_p, v_p));

    jointIdsToSolutions[0] = std::make_pair(id, theta);
    solutions[0] = jointIdsToSolutions;

    return solutions;
}

// -----------------------------------------------------------------------------

PadenKahanTwo::PadenKahanTwo(int _id1, int _id2, const MatrixExponential & _exp1, const MatrixExponential & _exp2, const KDL::Vector & _p)
  : id1(_id1),
    id2(_id2),
    exp1(_exp1),
    exp2(_exp2),
    p(_p)
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem::SolutionsVector PadenKahanTwo::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform)
{
    SolutionsVector solutions(2);
    JointIdsToSolutionsVector jointIdsToSolution1(2), jointIdsToSolution2(2);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * f;

    KDL::Vector u = f - exp1.getOrigin();
    KDL::Vector v = k - exp1.getOrigin();

    double axesDot = KDL::dot(exp1.getAxis(), exp2.getAxis());
    double axis1dot = KDL::dot(exp1.getAxis(), v);
    double axis2dot = KDL::dot(exp2.getAxis(), u);
    double den = std::pow(axesDot, 2) - 1;

    KDL::Vector axesCross = exp1.getAxis() * exp2.getAxis();

    double alpha = (axesDot * axis2dot - axis1dot) / den;
    double beta = (axesDot * axis1dot - axis2dot) / den;

    double gamma = std::sqrt(
            (std::pow(u.Norm(), 2) - std::pow(alpha, 2) - std::pow(beta, 2), - 2 * alpha * beta * axesDot) /
            std::pow(axesCross.Norm(), 2));

    KDL::Vector term1 = exp1.getOrigin() + alpha * exp1.getAxis() + beta * exp2.getAxis();
    KDL::Vector term2 = gamma * axesCross;

    KDL::Vector d = term1 + term2;
    KDL::Vector c = term1 - term2;

    KDL::Vector m = c - exp1.getOrigin();
    KDL::Vector n = d - exp1.getOrigin();

    KDL::Rotation axisPow1 = vectorPow2(exp1.getAxis());
    KDL::Rotation axisPow2 = vectorPow2(exp2.getAxis());

    KDL::Vector u_p = u - axisPow2 * u;
    KDL::Vector v_p = v - axisPow1 * v;

    KDL::Vector m1_p = m - axisPow1 * m;
    KDL::Vector m2_p = m - axisPow2 * m;

    KDL::Vector n1_p = n - axisPow1 * n;
    KDL::Vector n2_p = n - axisPow2 * n;

    double theta1_1 = std::atan2(KDL::dot(exp1.getAxis(), m1_p * v_p), KDL::dot(m1_p, v_p));
    double theta2_1 = std::atan2(KDL::dot(exp2.getAxis(), u_p * m2_p), KDL::dot(u_p, m2_p));

    double theta1_2 = std::atan2(KDL::dot(exp1.getAxis(), n1_p * v_p), KDL::dot(n1_p, v_p));
    double theta2_2 = std::atan2(KDL::dot(exp2.getAxis(), u_p * n2_p), KDL::dot(u_p, n2_p));

    jointIdsToSolution1[0] = std::make_pair(id1, theta1_1);
    jointIdsToSolution1[1] = std::make_pair(id2, theta1_1);

    solutions[0] = jointIdsToSolution1;

    jointIdsToSolution2[0] = std::make_pair(id1, theta1_2);
    jointIdsToSolution2[1] = std::make_pair(id2, theta1_2);

    solutions[1] = jointIdsToSolution2;

    return solutions;
}

// -----------------------------------------------------------------------------

PadenKahanThree::PadenKahanThree(int _id, const MatrixExponential & _exp, const KDL::Vector & _p, const KDL::Vector & _k)
    : id(_id),
      exp(_exp),
      p(_p),
      k(_k)
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem::SolutionsVector PadenKahanThree::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform)
{
    SolutionsVector solutions(2);
    JointIdsToSolutionsVector jointIdsToSolution1(1), jointIdsToSolution2(1);

    KDL::Vector f = pointTransform * p;
    KDL::Vector rhsAsVector = rhs * f - k;
    double delta = rhsAsVector.Norm();

    KDL::Vector u = f - exp.getOrigin();
    KDL::Vector v = k - exp.getOrigin();

    KDL::Rotation axisPow = vectorPow2(exp.getAxis());

    KDL::Vector u_p = u - axisPow * u;
    KDL::Vector v_p = v - axisPow * v;

    double alpha = std::atan2(KDL::dot(exp.getAxis(), u_p * v_p), KDL::dot(u_p, v_p));
    double delta_p_2 = std::pow(delta, 2) - std::pow(KDL::dot(exp.getAxis(), f - k), 2);

    double u_p_norm = u_p.Norm();
    double v_p_norm = v_p.Norm();

    double beta = std::acos((std::pow(u_p_norm, 2) + std::pow(v_p_norm, 2) - delta_p_2) / 2 * u_p_norm * v_p_norm);

    double theta1 = alpha + beta;
    double theta2 = alpha - beta;

    jointIdsToSolution1[0] = std::make_pair(id, theta1);
    jointIdsToSolution2[0] = std::make_pair(id, theta2);

    solutions[0] = jointIdsToSolution1;
    solutions[1] = jointIdsToSolution2;

    return solutions;
}

// -----------------------------------------------------------------------------

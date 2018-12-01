// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkSubproblems.hpp"

#include "ScrewTheoryTools.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

PardosOne::PardosOne(int _id, const MatrixExponential & _exp, const KDL::Vector & _p)
    : id(_id),
      exp(_exp),
      p(_p)
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem::SolutionsVector PardosOne::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform)
{
    SolutionsVector solutions(1);
    JointIdsToSolutionsVector jointIdsToSolutions(1);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * f;

    double theta = KDL::dot(exp.getAxis(), k - f);

    jointIdsToSolutions[0] = std::make_pair(id, normalizeAngle(theta));
    solutions[0] = jointIdsToSolutions;

    return solutions;
}

// -----------------------------------------------------------------------------

PardosTwo::PardosTwo(int _id1, int _id2, const MatrixExponential & _exp1, const MatrixExponential & _exp2, const KDL::Vector & _p)
    : id1(_id1),
      id2(_id2),
      exp1(_exp1),
      exp2(_exp2),
      p(_p)
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem::SolutionsVector PardosTwo::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform)
{
    SolutionsVector solutions(1);
    JointIdsToSolutionsVector jointIdsToSolutions(2);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * f;

    KDL::Vector crossPr1 = exp2.getAxis() * (f - k);
    KDL::Vector crossPr2 = exp2.getAxis() * exp1.getAxis();

    double crossPr1Norm = crossPr1.Norm();
    double crossPr2Norm = crossPr2.Norm();

    KDL::Vector c;

    if (KDL::dot(crossPr1, crossPr2) >= crossPr1Norm * crossPr2Norm)
    {
        c = k + (crossPr1Norm / crossPr2Norm) * exp1.getAxis();
    }
    else
    {
        c = k - (crossPr1Norm / crossPr2Norm) * exp1.getAxis();
    }

    double theta1 = KDL::dot(exp1.getAxis(), k - c);
    double theta2 = KDL::dot(exp2.getAxis(), c - f);

    jointIdsToSolutions[0] = std::make_pair(id1, normalizeAngle(theta1));
    jointIdsToSolutions[1] = std::make_pair(id2, normalizeAngle(theta2));

    solutions[0] = jointIdsToSolutions;

    return solutions;
}

// -----------------------------------------------------------------------------

PardosThree::PardosThree(int _id, const MatrixExponential & _exp, const KDL::Vector & _p, const KDL::Vector & _k)
    : id(_id),
      exp(_exp),
      p(_p),
      k(_k)
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem::SolutionsVector PardosThree::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform)
{
    SolutionsVector solutions(2);
    JointIdsToSolutionsVector jointIdsToSolution1(1), jointIdsToSolution2(1);

    KDL::Vector f = pointTransform * p;
    KDL::Vector rhsAsVector = rhs * f - k;
    double delta = rhsAsVector.Norm();

    KDL::Vector diff = k - f;

    double dotPr = KDL::dot(exp.getAxis(), diff);
    double sq = std::sqrt(std::pow(dotPr, 2) - std::pow(diff.Norm(), 2) + std::pow(delta, 2));

    double theta1 = dotPr + sq;
    double theta2 = dotPr - sq;

    jointIdsToSolution1[0] = std::make_pair(id, normalizeAngle(theta1));
    jointIdsToSolution2[0] = std::make_pair(id, normalizeAngle(theta2));

    solutions[0] = jointIdsToSolution1;
    solutions[1] = jointIdsToSolution2;

    return solutions;
}

// -----------------------------------------------------------------------------

PardosFour::PardosFour(int _id1, int _id2, const MatrixExponential & _exp1, const MatrixExponential & _exp2, const KDL::Vector & _p)
    : id1(_id1),
      id2(_id2),
      exp1(_exp1),
      exp2(_exp2),
      p(_p)
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem::SolutionsVector PardosFour::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform)
{
    SolutionsVector solutions(2);
    JointIdsToSolutionsVector jointIdsToSolution1(2), jointIdsToSolution2(2);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * f;

    KDL::Vector u = f - exp2.getOrigin();
    KDL::Vector v = k - exp1.getOrigin();

    KDL::Rotation axisPow1 = vectorPow2(exp1.getAxis());
    KDL::Rotation axisPow2 = vectorPow2(exp2.getAxis());

    KDL::Vector u_p = u - axisPow2 * u;
    KDL::Vector v_p = v - axisPow1 * v;

    KDL::Vector c1 = exp1.getOrigin() + v - v_p;
    KDL::Vector c2 = exp2.getOrigin() + u - u_p;

    KDL::Vector c_diff = c2 - c1;
    double c_norm = c_diff.Norm();

    KDL::Vector omega_a = c_diff / c_norm;
    KDL::Vector omega_h = exp1.getAxis() * omega_a;

    double a = (std::pow(c_norm, 2) - std::pow(u_p.Norm(), 2) + std::pow(v_p.Norm(), 2)) / 2 * c_norm;
    double h = std::sqrt(std::pow(v_p.Norm(), 2) - std::pow(a, 2));

    KDL::Vector term1 = c1 + a * omega_a;
    KDL::Vector term2 = h * omega_h;

    KDL::Vector c = term1 + term2;
    KDL::Vector d = term1 - term2;

    KDL::Vector m1 = c - exp1.getOrigin();
    KDL::Vector m2 = c - exp2.getOrigin();

    KDL::Vector n1 = d - exp1.getOrigin();
    KDL::Vector n2 = d - exp2.getOrigin();

    KDL::Vector m1_p = m1 - axisPow1 * m1;
    KDL::Vector m2_p = m2 - axisPow2 * m2;

    KDL::Vector n1_p = n1 - axisPow1 * n1;
    KDL::Vector n2_p = n2 - axisPow2 * n2;

    double theta1_1 = std::atan2(KDL::dot(exp1.getAxis(), m1_p * v_p), KDL::dot(m1_p, v_p));
    double theta2_1 = std::atan2(KDL::dot(exp2.getAxis(), u_p * m2_p), KDL::dot(u_p, m2_p));

    double theta1_2 = std::atan2(KDL::dot(exp1.getAxis(), n1_p * v_p), KDL::dot(n1_p, v_p));
    double theta2_2 = std::atan2(KDL::dot(exp2.getAxis(), u_p * n2_p), KDL::dot(u_p, n2_p));

    jointIdsToSolution1[0] = std::make_pair(id1, normalizeAngle(theta1_1));
    jointIdsToSolution1[1] = std::make_pair(id2, normalizeAngle(theta2_1));

    solutions[0] = jointIdsToSolution1;

    jointIdsToSolution2[0] = std::make_pair(id1, normalizeAngle(theta1_2));
    jointIdsToSolution2[1] = std::make_pair(id2, normalizeAngle(theta2_2));

    solutions[1] = jointIdsToSolution2;

    return solutions;
}

// -----------------------------------------------------------------------------

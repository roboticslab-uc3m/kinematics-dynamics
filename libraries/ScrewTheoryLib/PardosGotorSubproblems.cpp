// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkSubproblems.hpp"

#include "ScrewTheoryTools.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    KDL::Vector computeNormal(const MatrixExponential & exp1, const MatrixExponential & exp2)
    {
        KDL::Vector diff = exp2.getOrigin() - exp1.getOrigin();
        KDL::Vector normal = (exp1.getAxis() * diff) * exp1.getAxis();
        normal.Normalize();
        return vectorPow2(normal) * diff;
    }
}

// -----------------------------------------------------------------------------

PardosGotorOne::PardosGotorOne(int _id, const MatrixExponential & _exp, const KDL::Vector & _p)
    : id(_id),
      exp(_exp),
      p(_p)
{}

// -----------------------------------------------------------------------------

bool PardosGotorOne::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const
{
    solutions.resize(PardosGotorOne::solutions());
    JointIdsToSolutions jointIdsToSolutions(1);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * p;

    KDL::Vector diff = k - f;
    double theta = KDL::dot(exp.getAxis(), diff);

    jointIdsToSolutions[0] = std::make_pair(id, theta);
    solutions[0] = jointIdsToSolutions;

    return true;
}

// -----------------------------------------------------------------------------

PardosGotorTwo::PardosGotorTwo(int _id1, int _id2, const MatrixExponential & _exp1, const MatrixExponential & _exp2, const KDL::Vector & _p)
    : id1(_id1),
      id2(_id2),
      exp1(_exp1),
      exp2(_exp2),
      p(_p),
      crossPr2(exp2.getAxis() * exp1.getAxis()),
      crossPr2Norm(crossPr2.Norm())
{}

// -----------------------------------------------------------------------------

bool PardosGotorTwo::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const
{
    solutions.resize(PardosGotorTwo::solutions());
    JointIdsToSolutions jointIdsToSolutions(2);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * p;

    KDL::Vector crossPr1 = exp2.getAxis() * (f - k);
    double crossPr1Norm = crossPr1.Norm();

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

    jointIdsToSolutions[0] = std::make_pair(id1, theta1);
    jointIdsToSolutions[1] = std::make_pair(id2, theta2);

    solutions[0] = jointIdsToSolutions;

    return true;
}

// -----------------------------------------------------------------------------

PardosGotorThree::PardosGotorThree(int _id, const MatrixExponential & _exp, const KDL::Vector & _p, const KDL::Vector & _k)
    : id(_id),
      exp(_exp),
      p(_p),
      k(_k)
{}

// -----------------------------------------------------------------------------

bool PardosGotorThree::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const
{
    solutions.resize(PardosGotorThree::solutions());
    JointIdsToSolutions jointIdsToSolution1(1), jointIdsToSolution2(1);

    KDL::Vector f = pointTransform * p;
    KDL::Vector rhsAsVector = rhs * p - k;
    double delta = rhsAsVector.Norm();

    KDL::Vector diff = k - f;

    double dotPr = KDL::dot(exp.getAxis(), diff);
    double sq2 = std::pow(dotPr, 2) - std::pow(diff.Norm(), 2) + std::pow(delta, 2);
    bool sq2_zero = KDL::Equal(sq2, 0.0);

    jointIdsToSolution1[0].first = jointIdsToSolution2[0].first = id;

    bool ret;

    if (!sq2_zero && sq2 > 0)
    {
        double sq = std::sqrt(std::abs(sq2));
        jointIdsToSolution1[0].second = dotPr + sq;
        jointIdsToSolution2[0].second = dotPr - sq;
        ret = true;
    }
    else
    {
        KDL::Vector proy = vectorPow2(exp.getAxis()) * diff;
        jointIdsToSolution1[0].second = jointIdsToSolution2[0].second = proy.Norm();
        ret = sq2_zero;
    }

    solutions[0] = jointIdsToSolution1;
    solutions[1] = jointIdsToSolution2;

    return ret;
}

// -----------------------------------------------------------------------------

PardosGotorFour::PardosGotorFour(int _id1, int _id2, const MatrixExponential & _exp1, const MatrixExponential & _exp2, const KDL::Vector & _p)
    : id1(_id1),
      id2(_id2),
      exp1(_exp1),
      exp2(_exp2),
      p(_p),
      n(computeNormal(exp1, exp2)),
      axisPow(vectorPow2(exp1.getAxis())) // same as exp2.getAxis()
{}

// -----------------------------------------------------------------------------

bool PardosGotorFour::solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const
{
    solutions.resize(PardosGotorFour::solutions());
    JointIdsToSolutions jointIdsToSolution1(2), jointIdsToSolution2(2);

    KDL::Vector f = pointTransform * p;
    KDL::Vector k = rhs * p;

    KDL::Vector u = f - exp2.getOrigin();
    KDL::Vector v = k - exp1.getOrigin();

    KDL::Vector u_p = u - axisPow * u;
    KDL::Vector v_p = v - axisPow * v;

    KDL::Vector c1 = exp1.getOrigin() + v - v_p;
    KDL::Vector c2 = exp2.getOrigin() + u - u_p;

    KDL::Vector c_diff = c2 - c1;
    bool samePlane = KDL::Equal(c_diff, n);

    if (!samePlane)
    {
        c_diff = n; // proyection of c_diff onto the perpendicular plane
        c1 = c2 - c_diff; // c1 on the intersecion of axis 1 and the normal plane to both axes
    }

    double c_norm = c_diff.Norm();
    double u_p_norm = u_p.Norm();
    double v_p_norm = v_p.Norm();

    double c_test = u_p_norm + v_p_norm - c_norm;
    bool c_zero = KDL::Equal(c_test, 0.0);

    jointIdsToSolution1[0].first = jointIdsToSolution2[0].first = id1;
    jointIdsToSolution1[1].first = jointIdsToSolution2[1].first = id2;

    bool ret;

    if (!c_zero && c_test > 0.0)
    {
        KDL::Vector omega_a = c_diff / c_norm;
        KDL::Vector omega_h = exp1.getAxis() * omega_a;

        double a = (std::pow(c_norm, 2) - std::pow(u_p_norm, 2) + std::pow(v_p_norm, 2)) / (2 * c_norm);
        double h = std::sqrt(std::abs(std::pow(v_p.Norm(), 2) - std::pow(a, 2)));

        KDL::Vector term1 = c1 + a * omega_a;
        KDL::Vector term2 = h * omega_h;

        KDL::Vector c = term1 + term2;
        KDL::Vector d = term1 - term2;

        KDL::Vector m1 = c - exp1.getOrigin();
        KDL::Vector m2 = c - exp2.getOrigin();

        KDL::Vector n1 = d - exp1.getOrigin();
        KDL::Vector n2 = d - exp2.getOrigin();

        KDL::Vector m1_p = m1 - axisPow * m1;
        KDL::Vector m2_p = m2 - axisPow * m2;

        KDL::Vector n1_p = n1 - axisPow * n1;
        KDL::Vector n2_p = n2 - axisPow * n2;

        double theta1_1 = std::atan2(KDL::dot(exp1.getAxis(), m1_p * v_p), KDL::dot(m1_p, v_p));
        double theta2_1 = std::atan2(KDL::dot(exp2.getAxis(), u_p * m2_p), KDL::dot(u_p, m2_p));

        double theta1_2 = std::atan2(KDL::dot(exp1.getAxis(), n1_p * v_p), KDL::dot(n1_p, v_p));
        double theta2_2 = std::atan2(KDL::dot(exp2.getAxis(), u_p * n2_p), KDL::dot(u_p, n2_p));

        jointIdsToSolution1[0].second = normalizeAngle(theta1_1);
        jointIdsToSolution1[1].second = normalizeAngle(theta2_1);

        jointIdsToSolution2[0].second = normalizeAngle(theta1_2);
        jointIdsToSolution2[1].second = normalizeAngle(theta2_2);

        ret = samePlane && KDL::Equal(m1_p.Norm(), v_p_norm);
    }
    else
    {
        double theta1 = std::atan2(KDL::dot(exp1.getAxis(), c_diff * v_p), KDL::dot(c_diff, v_p));
        double theta2 = std::atan2(KDL::dot(exp2.getAxis(), u_p * c_diff), KDL::dot(-c_diff, u_p));

        jointIdsToSolution1[0].second = jointIdsToSolution2[0].second = normalizeAngle(theta1);
        jointIdsToSolution1[1].second = jointIdsToSolution2[1].second = normalizeAngle(theta2);

        ret = c_zero;
    }

    solutions[0] = jointIdsToSolution1;
    solutions[1] = jointIdsToSolution2;

    return ret;
}

// -----------------------------------------------------------------------------

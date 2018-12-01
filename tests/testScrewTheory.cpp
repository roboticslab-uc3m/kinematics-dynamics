#include "gtest/gtest.h"

#include <algorithm>
#include <functional>
#include <vector>
#include <utility>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/utilities/utility.h>

#include "MatrixExponential.hpp"
#include "ProductOfExponentials.hpp"
#include "ScrewTheoryIkSubproblems.hpp"

namespace roboticslab
{

/**
 * @brief Tests classes related to Screw Theory.
 */
class ScrewTheoryTest : public testing::Test
{
public:
    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
    }

    static KDL::Chain makeTeoRightArmKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        KDL::Chain chain;

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0,  KDL::PI / 2,     0.0,          0.0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0,  KDL::PI / 2,     0.0, -KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0,  KDL::PI / 2, 0.32901, -KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0, -KDL::PI / 2,     0.0,          0.0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0,  KDL::PI / 2,   0.202,          0.0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.187496, -KDL::PI / 2,     0.0,  KDL::PI / 2)));

        return chain;
    }

    static PoeExpression makeTeoRightArmKinematicsFromPoE()
    {
        std::vector<MatrixExponential> exps;
        exps.reserve(6);

        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector( 0,  0, 1), KDL::Vector::Zero()));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector( 0, -1, 0), KDL::Vector::Zero()));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(-1,  0, 0), KDL::Vector::Zero()));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector( 0,  0, 1), KDL::Vector(-0.32901, 0, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(-1,  0, 0), KDL::Vector(-0.32901, 0, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector( 0,  0, 1), KDL::Vector(-0.53101, 0, 0)));

        KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI / 2) * KDL::Rotation::RotZ(KDL::PI), KDL::Vector(-0.718506, 0, 0));

        return PoeExpression(exps, H_S_T);
    }

    static void checkSolutions(const ScrewTheoryIkSubproblem::SolutionsVector & actual, const ScrewTheoryIkSubproblem::SolutionsVector & expected)
    {
        ScrewTheoryIkSubproblem::JointIdsToSolutionsVector actualSorted, expectedSorted;

        for (int i = 0; i < actual.size(); i++)
        {
            for (int j = 0; j < actual[i].size(); j++)
            {
                actualSorted.push_back(actual[i][j]);
            }
        }

        for (int i = 0; i < expected.size(); i++)
        {
            for (int j = 0; j < expected[i].size(); j++)
            {
                expectedSorted.push_back(expected[i][j]);
            }
        }

        ASSERT_EQ(actualSorted.size(), expectedSorted.size());

        std::sort(actualSorted.begin(), actualSorted.end(), solutionComparator);
        std::sort(expectedSorted.begin(), expectedSorted.end(), solutionComparator);

        for (int i = 0; i < actualSorted.size(); i++)
        {
            ASSERT_EQ(actualSorted[i].first, expectedSorted[i].first);
            ASSERT_NEAR(actualSorted[i].second, expectedSorted[i].second, KDL::epsilon);
        }
    }

private:
    static struct compare_solutions : public std::binary_function<std::pair<int, double>, std::pair<int, double>, bool>
    {
        bool operator()(const std::pair<int, double> & lhs, const std::pair<int, double> & rhs)
        {
            return !(lhs.first > rhs.first) && (lhs.first < rhs.first || lhs.second < rhs.second);
        }
    }
    solutionComparator;
};

TEST_F(ScrewTheoryTest, MatrixExponentialInit)
{
    MatrixExponential::motion motionType = MatrixExponential::ROTATION;
    KDL::Vector axis(1, 0, 0);
    KDL::Vector origin(1, 1, 1);

    MatrixExponential exp(MatrixExponential::ROTATION, axis, origin);

    ASSERT_EQ(exp.getMotionType(), motionType);
    ASSERT_EQ(exp.getAxis(), axis);
    ASSERT_EQ(exp.getOrigin(), origin);
}

TEST_F(ScrewTheoryTest, MatrixExponentialRotation)
{
    double theta = KDL::PI / 2;
    MatrixExponential exp(MatrixExponential::ROTATION, KDL::Vector(0, 0, 1), KDL::Vector(1, 0, 0));

    KDL::Frame actual = exp.asFrame(theta);
    KDL::Frame expected(KDL::Rotation::RotZ(theta), KDL::Vector(1, -1, 0));
    ASSERT_EQ(actual, expected);
}

TEST_F(ScrewTheoryTest, MatrixExponentialTranslation)
{
    KDL::Vector axis(0, 0, 1);
    double theta = KDL::PI;
    MatrixExponential exp(MatrixExponential::TRANSLATION, axis);

    KDL::Frame actual = exp.asFrame(theta);
    KDL::Frame expected(KDL::Rotation::Identity(), theta * axis);

    ASSERT_EQ(actual, expected);
}

TEST_F(ScrewTheoryTest, ProductOfExponentialsFromChain)
{
    KDL::Chain chain = makeTeoRightArmKinematicsFromDH();
    PoeExpression poe = PoeExpression::fromChain(chain);
    ASSERT_EQ(poe.size(), chain.getNrOfJoints());

    KDL::ChainFkSolverPos_recursive fkSolver(chain);

    KDL::Frame H_S_T_0;
    ASSERT_EQ(fkSolver.JntToCart(KDL::JntArray(chain.getNrOfJoints()), H_S_T_0), KDL::SolverI::E_NOERROR);
    ASSERT_EQ(poe.getTransform(), H_S_T_0);

    KDL::JntArray q(chain.getNrOfJoints());

    for (int i = 0; i < chain.getNrOfJoints(); i++)
    {
        q(i) = KDL::PI / 2;
    }

    KDL::Frame H_S_T_q_DH, H_S_T_q_ST;
    ASSERT_EQ(fkSolver.JntToCart(q, H_S_T_q_DH), KDL::SolverI::E_NOERROR);
    ASSERT_TRUE(poe.evaluate(q, H_S_T_q_ST));
    ASSERT_EQ(H_S_T_q_ST, H_S_T_q_DH);
}

TEST_F(ScrewTheoryTest, ProductOfExponentialsToChain)
{
    PoeExpression poe = makeTeoRightArmKinematicsFromPoE();

    KDL::Chain chainDH = makeTeoRightArmKinematicsFromDH();
    KDL::Chain chainST = poe.toChain();

    KDL::ChainFkSolverPos_recursive fkSolverDH(chainDH);
    KDL::ChainFkSolverPos_recursive fkSolverST(chainST);

    KDL::JntArray q(poe.size());

    for (int i = 0; i < poe.size(); i++)
    {
        q(i) = KDL::PI / 2;
    }

    KDL::Frame H_S_T_q_DH, H_S_T_q_ST;
    ASSERT_EQ(fkSolverDH.JntToCart(q, H_S_T_q_DH), KDL::SolverI::E_NOERROR);
    ASSERT_EQ(fkSolverST.JntToCart(q, H_S_T_q_ST), KDL::SolverI::E_NOERROR);
    ASSERT_EQ(H_S_T_q_ST, H_S_T_q_DH);
}

TEST_F(ScrewTheoryTest, ProductOfExponentialsIntegrity)
{
    PoeExpression poe = makeTeoRightArmKinematicsFromPoE();

    KDL::Chain chain = poe.toChain();
    KDL::ChainFkSolverPos_recursive fkSolver(chain);

    KDL::JntArray q(poe.size());

    for (int i = 0; i < poe.size(); i++)
    {
        q(i) = KDL::PI / 2;
    }

    KDL::Frame H_S_T_q_DH, H_S_T_q_ST;
    ASSERT_EQ(fkSolver.JntToCart(q, H_S_T_q_DH), KDL::SolverI::E_NOERROR);

    PoeExpression poeFromChain = PoeExpression::fromChain(chain);
    ASSERT_TRUE(poeFromChain.evaluate(q, H_S_T_q_ST));
    ASSERT_EQ(H_S_T_q_ST, H_S_T_q_DH);
}

TEST_F(ScrewTheoryTest, PadenKahanOne)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(1, 1, 1);

    MatrixExponential exp(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(1, 0, 0));
    PadenKahanOne pk1(0, exp, p);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::SolutionsVector actual = pk1.solve(rhs, KDL::Frame::Identity());

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 1);

    ScrewTheoryIkSubproblem::SolutionsVector expected(1);
    ScrewTheoryIkSubproblem::JointIdsToSolutionsVector sol(1);

    sol[0] = std::make_pair(0, KDL::PI / 2);
    expected[0] = sol;

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PadenKahanTwo)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(1, -1, 1);

    MatrixExponential exp1(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(1, 0, 0));
    MatrixExponential exp2(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(1, 0, 0));
    PadenKahanTwo pk2(0, 1, exp1, exp2, p);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::SolutionsVector actual = pk2.solve(rhs, KDL::Frame::Identity());

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 2);
    ASSERT_EQ(actual[1].size(), 2);

    ScrewTheoryIkSubproblem::SolutionsVector expected(2);
    ScrewTheoryIkSubproblem::JointIdsToSolutionsVector sols1(2), sols2(2);

    sols1[0] = std::make_pair(0, KDL::PI / 2);
    sols1[1] = std::make_pair(1, KDL::PI / 2);

    sols2[0] = std::make_pair(0, KDL::PI);
    sols2[1] = std::make_pair(1, -KDL::PI / 2);

    expected[0] = sols1;
    expected[1] = sols2;

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PadenKahanThree)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(2, 1, 1);
    KDL::Vector delta(-1, 0, 0);

    MatrixExponential exp(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(1, 0, 0));
    PadenKahanThree pk3(0, exp, p, k);

    KDL::Frame rhs(delta - (p - k));
    ScrewTheoryIkSubproblem::SolutionsVector actual = pk3.solve(rhs, KDL::Frame::Identity());

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 1);
    ASSERT_EQ(actual[1].size(), 1);

    ScrewTheoryIkSubproblem::SolutionsVector expected(2);
    ScrewTheoryIkSubproblem::JointIdsToSolutionsVector sol1(1), sol2(1);

    sol1[0] = std::make_pair(0, KDL::PI / 2);
    sol2[0] = std::make_pair(0, KDL::PI);

    expected[0] = sol1;
    expected[1] = sol2;

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosOne)
{
    KDL::Vector p(1, 0, 0);
    KDL::Vector k(1, 1, 0);

    MatrixExponential exp(MatrixExponential::TRANSLATION, KDL::Vector(0, 1, 0));
    PardosOne pg1(0, exp, p);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::SolutionsVector actual = pg1.solve(rhs, KDL::Frame::Identity());

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 1);

    ScrewTheoryIkSubproblem::SolutionsVector expected(1);
    ScrewTheoryIkSubproblem::JointIdsToSolutionsVector sol(1);

    sol[0] = std::make_pair(0, 1.0);
    expected[0] = sol;

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosTwo)
{
    KDL::Vector p(1, 1, 0);
    KDL::Vector k(2, 3, 0);

    MatrixExponential exp1(MatrixExponential::TRANSLATION, KDL::Vector(0, 1, 0));
    MatrixExponential exp2(MatrixExponential::TRANSLATION, KDL::Vector(1, 0, 0));
    PardosTwo pg2(0, 1, exp1, exp2, p);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::SolutionsVector actual = pg2.solve(rhs, KDL::Frame::Identity());

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 2);

    ScrewTheoryIkSubproblem::SolutionsVector expected(1);
    ScrewTheoryIkSubproblem::JointIdsToSolutionsVector sols(2);

    sols[0] = std::make_pair(0, k.y() - p.y());
    sols[1] = std::make_pair(1, k.x() - p.x());

    expected[0] = sols;

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosThree)
{
    KDL::Vector p(1, 0, 0);
    KDL::Vector k(1, 2, 0);
    KDL::Vector delta(0, 1, 0);

    MatrixExponential exp(MatrixExponential::TRANSLATION, KDL::Vector(0, 1, 0));
    PardosThree pg3(0, exp, p, k);

    KDL::Frame rhs(delta - (p - k));
    ScrewTheoryIkSubproblem::SolutionsVector actual = pg3.solve(rhs, KDL::Frame::Identity());

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 1);
    ASSERT_EQ(actual[1].size(), 1);

    ScrewTheoryIkSubproblem::SolutionsVector expected(2);
    ScrewTheoryIkSubproblem::JointIdsToSolutionsVector sol1(1), sol2(1);

    sol1[0] = std::make_pair(0, k.y() - p.y() - delta.y());
    sol2[0] = std::make_pair(0, k.y() - p.y() + delta.y());

    expected[0] = sol1;
    expected[1] = sol2;

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosFour)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(3, 1, 1);

    MatrixExponential exp1(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(2, 0, 0));
    MatrixExponential exp2(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(1, 0, 0));
    PardosFour pg4(0, 1, exp1, exp2, p);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::SolutionsVector actual = pg4.solve(rhs, KDL::Frame::Identity());

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 2);
    ASSERT_EQ(actual[1].size(), 2);

    ScrewTheoryIkSubproblem::SolutionsVector expected(2);
    ScrewTheoryIkSubproblem::JointIdsToSolutionsVector sols1(2), sols2(2);

    sols1[0] = std::make_pair(0, KDL::PI / 2);
    sols1[1] = std::make_pair(1, KDL::PI / 2);

    sols2[0] = std::make_pair(0, KDL::PI);
    sols2[1] = std::make_pair(1, -KDL::PI / 2);

    expected[0] = sols1;
    expected[1] = sols2;

    checkSolutions(actual, expected);
}

}  // namespace roboticslab

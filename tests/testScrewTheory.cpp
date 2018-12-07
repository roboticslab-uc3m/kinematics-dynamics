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
#include "ScrewTheoryIkProblem.hpp"
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

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(       0,  KDL::PI / 2,       0,            0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(       0,  KDL::PI / 2,       0, -KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(       0,  KDL::PI / 2, 0.32901, -KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(       0, -KDL::PI / 2,       0,            0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(       0,  KDL::PI / 2,   0.202,            0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.187496, -KDL::PI / 2,       0,  KDL::PI / 2)));

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

    static KDL::Chain makeAbbIrb120KinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotX(-KDL::PI / 2))));

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(   0,  KDL::PI / 2,  0.29,            0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.27,            0,     0,  KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.07,  KDL::PI / 2,     0,            0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(   0, -KDL::PI / 2, 0.302,            0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(   0,  KDL::PI / 2,     0, -KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(   0,            0,  0.16,            0)));

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotZ(KDL::PI / 2))));

        return chain;
    }

    static PoeExpression makeAbbIrb120KinematicsFromPoE()
    {
        std::vector<MatrixExponential> exps;
        exps.reserve(6);

        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0,  1, 0), KDL::Vector::Zero()));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(    0, 0.29, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(    0, 0.56, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(1,  0, 0), KDL::Vector(0.302, 0.63, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(0.302, 0.63, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, -1, 0), KDL::Vector(0.302, 0.63, 0)));

        KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI / 2) * KDL::Rotation::RotZ(KDL::PI / 2), KDL::Vector(0.302, 0.47, 0));

        return PoeExpression(exps, H_S_T);
    }

    static KDL::Chain makePumaKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotX(-KDL::PI / 2))));

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0,  KDL::PI / 2, 2,  KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(1,            0, 0,  KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0,  KDL::PI / 2, 2,            0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0,  KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0,            0, 1,            0)));

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotZ(-KDL::PI / 2))));

        return chain;
    }

    static PoeExpression makePumaKinematicsFromPoE()
    {
        std::vector<MatrixExponential> exps;
        exps.reserve(6);

        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector::Zero()));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 2, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 3, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(0, 3, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 5, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 0, 1), KDL::Vector(0, 5, 0)));

        KDL::Frame H_S_T(KDL::Vector(0, 5, 1));

        return PoeExpression(exps, H_S_T);
    }

    static KDL::Chain makeStanfordKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        const KDL::Joint translZ(KDL::Joint::TransZ);

        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotX(-KDL::PI / 2))));

        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0,  KDL::PI / 2, 2,  KDL::PI / 2)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0,            0)));
        chain.addSegment(KDL::Segment(translZ, KDL::Frame::DH(0,            0, 0,            0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0,  KDL::PI / 2, 3,            0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0,  KDL::PI / 2)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0,            0, 1,            0)));

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotZ(-KDL::PI / 2))));

        return chain;
    }

    static PoeExpression makeStanfordKinematicsFromPoE()
    {
        std::vector<MatrixExponential> exps;
        exps.reserve(6);

        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector::Zero()));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 2, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::TRANSLATION, KDL::Vector(0, 1, 0)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(0, 2, 0)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 5, 0)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, 0, 1), KDL::Vector(0, 5, 0)));

        KDL::Frame H_S_T(KDL::Vector(0, 5, 1));

        return PoeExpression(exps, H_S_T);
    }

    static KDL::Chain makeAbbIrb910scKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        const KDL::Joint translZ(KDL::Joint::TransZ);

        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotX(-KDL::PI / 2))));

        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH( 0.4,       0,      0, 0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0.25, KDL::PI,      0, 0)));
        chain.addSegment(KDL::Segment(translZ, KDL::Frame::DH(   0,       0,      0, 0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(   0,       0, -0.125, 0)));

        return chain;
    }

    static PoeExpression makeAbbIrb910scKinematicsFromPoE()
    {
        std::vector<MatrixExponential> exps;
        exps.reserve(4);

        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  1, 0), KDL::Vector::Zero()));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  1, 0), KDL::Vector(0.4, 0, 0)));
        exps.push_back(MatrixExponential(MatrixExponential::TRANSLATION, KDL::Vector(0, -1, 0)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, -1, 0), KDL::Vector(0.65, 0, 0)));

        KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI / 2), KDL::Vector(0.65, 0.125, 0));

        return PoeExpression(exps, H_S_T);
    }

    static KDL::Chain makeAbbIrb6620lxFromDh()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        const KDL::Joint translZ(KDL::Joint::TransZ);

        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(1.088, 2.5, 0))));

        chain.addSegment(KDL::Segment(translZ, KDL::Frame::DH( 0.38,            0,     0,           0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0.975,            0,     0,           0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(  0.2,  KDL::PI / 2,     0,           0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(    0, -KDL::PI / 2, 0.887,           0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(    0,  KDL::PI / 2,     0, KDL::PI / 2)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(    0,            0, 0.357,           0)));

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotZ(-KDL::PI / 2))));

        return chain;
    }

    static PoeExpression makeAbbIrb6620lxFromPoE()
    {
        std::vector<MatrixExponential> exps;
        exps.reserve(6);

        exps.push_back(MatrixExponential(MatrixExponential::TRANSLATION, KDL::Vector(0,  0, 1)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(1.468,   2.5, 0)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(2.443,   2.5, 0)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, -1, 0), KDL::Vector(2.643, 1.613, 0)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(2.643, 1.613, 0)));
        exps.push_back(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(1,  0, 0), KDL::Vector(2.643, 1.613, 0)));

        KDL::Frame H_S_T(KDL::Rotation::RotY(KDL::PI / 2), KDL::Vector(3, 1.613, 0));

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

    static void checkRobotKinematics(const KDL::Chain & chain, const PoeExpression & poe)
    {
        ASSERT_EQ(poe.size(), chain.getNrOfJoints());

        KDL::ChainFkSolverPos_recursive fkSolver(chain);
        KDL::Frame H_S_T_0_DH, H_S_T_0_ST;

        ASSERT_EQ(fkSolver.JntToCart(KDL::JntArray(chain.getNrOfJoints()), H_S_T_0_DH), KDL::SolverI::E_NOERROR);
        ASSERT_TRUE(poe.evaluate(KDL::JntArray(poe.size()), H_S_T_0_ST));
        ASSERT_EQ(H_S_T_0_ST, H_S_T_0_DH);

        KDL::JntArray q(chain.getNrOfJoints());

        for (int i = 0; i < chain.getNrOfJoints(); i++)
        {
            q(i) = KDL::PI / 2;
        }

        KDL::Frame H_S_T_q_DH, H_S_T_q_ST;
        ASSERT_EQ(fkSolver.JntToCart(q, H_S_T_q_DH), KDL::SolverI::E_NOERROR);
        ASSERT_TRUE(poe.evaluate(q, H_S_T_q_ST));
        ASSERT_EQ(H_S_T_q_ST, H_S_T_q_DH);

        ScrewTheoryIkProblemBuilder builder(poe);
        ScrewTheoryIkProblem * ikProblem = builder.build();

        ASSERT_TRUE(ikProblem);

        std::vector<KDL::JntArray> solutions;
        ASSERT_TRUE(ikProblem->solve(H_S_T_q_ST, solutions));
        delete ikProblem;

        for (int i = 0; i < solutions.size(); i++)
        {
            KDL::Frame H_S_T_q_ST_validate;
            ASSERT_TRUE(poe.evaluate(solutions[i], H_S_T_q_ST_validate));
            ASSERT_EQ(H_S_T_q_ST_validate, H_S_T_q_ST);
        }
    }

private:

    static struct compare_solutions
        : public std::binary_function<const std::pair<int, double> &, const std::pair<int, double> &, bool>
    {
        result_type operator()(first_argument_type lhs, second_argument_type rhs)
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

TEST_F(ScrewTheoryTest, ProductOfExponentialsReverse)
{
    PoeExpression poe = makeTeoRightArmKinematicsFromPoE();
    PoeExpression poeReversed = poe.reverse();

    ASSERT_EQ(poeReversed.size(), poe.size());
    ASSERT_EQ(poeReversed.getTransform(), poe.getTransform().Inverse());

    KDL::Frame H_S_T_0, H_S_T_0_reversed;

    ASSERT_TRUE(poe.evaluate(KDL::JntArray(poe.size()), H_S_T_0));
    ASSERT_TRUE(poeReversed.evaluate(KDL::JntArray(poeReversed.size()), H_S_T_0_reversed));
    ASSERT_EQ(H_S_T_0_reversed, H_S_T_0.Inverse());

    KDL::JntArray q(poe.size());

    for (int i = 0; i < poe.size(); i++)
    {
        q(i) = KDL::PI / 2;
    }

    KDL::Frame H_S_T_q;
    ASSERT_TRUE(poe.evaluate(q, H_S_T_q));

    for (int i = 0; i < poe.size(); i++)
    {
        q(i) = -KDL::PI / 2;
    }

    KDL::Frame H_S_T_q_reversed;
    ASSERT_TRUE(poeReversed.evaluate(q, H_S_T_q_reversed));

    ASSERT_EQ(H_S_T_q_reversed, H_S_T_q.Inverse());
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
    KDL::Vector r(1, 0, 0);

    MatrixExponential exp1(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), r);
    MatrixExponential exp2(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), r);
    PadenKahanTwo pk2(0, 1, exp1, exp2, p, r);

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

TEST_F(ScrewTheoryTest, AbbIrb120Kinematics)
{
    KDL::Chain chain = makeAbbIrb120KinematicsFromDH();
    PoeExpression poe = makeAbbIrb120KinematicsFromPoE();

    checkRobotKinematics(chain, poe);
}

TEST_F(ScrewTheoryTest, PumaKinematics)
{
    KDL::Chain chain = makePumaKinematicsFromDH();
    PoeExpression poe = makePumaKinematicsFromPoE();

    checkRobotKinematics(chain, poe);
}

TEST_F(ScrewTheoryTest, StanfordKinematics)
{
    KDL::Chain chain = makeStanfordKinematicsFromDH();
    PoeExpression poe = makeStanfordKinematicsFromPoE();

    checkRobotKinematics(chain, poe);
}

TEST_F(ScrewTheoryTest, AbbIrb910scKinematics)
{
    KDL::Chain chain = makeAbbIrb910scKinematicsFromDH();
    PoeExpression poe = makeAbbIrb910scKinematicsFromPoE();

    checkRobotKinematics(chain, poe);
}

TEST_F(ScrewTheoryTest, AbbIrb6620lxKinematics)
{
    KDL::Chain chain = makeAbbIrb6620lxFromDh();
    PoeExpression poe = makeAbbIrb6620lxFromPoE();

    checkRobotKinematics(chain, poe);
}

TEST_F(ScrewTheoryTest, TeoRightArmKinematics)
{
    KDL::Chain chain = makeTeoRightArmKinematicsFromDH();
    PoeExpression poe = makeTeoRightArmKinematicsFromPoE();

    checkRobotKinematics(chain, poe);
}

}  // namespace roboticslab

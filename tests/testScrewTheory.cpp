#include "gtest/gtest.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/utilities/utility.h>

#include "ConfigurationSelector.hpp"
#include "MatrixExponential.hpp"
#include "ProductOfExponentials.hpp"
#include "ScrewTheoryIkProblem.hpp"
#include "ScrewTheoryIkSubproblems.hpp"

namespace roboticslab::test
{

/**
 * @ingroup kinematics-dynamics-tests
 * @brief Tests classes related to Screw Theory.
 */
class ScrewTheoryTest : public testing::Test
{
public:
    void SetUp() override
    {}

    void TearDown() override
    {}

    static KDL::JntArray fillJointValues(int size, double value)
    {
        KDL::JntArray q(size);

        for (int i = 0; i < size; i++)
        {
            q(i) = value;
        }

        return q;
    }

    static KDL::Chain makeTeoRightArmKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        KDL::Chain chain;

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI_2,        0,          0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI_2,        0, -KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI_2, -0.32901, -KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0,  KDL::PI_2,        0,          0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI_2,   -0.215,          0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09,          0,        0, -KDL::PI_2)));

        return chain;
    }

    static PoeExpression makeTeoRightArmKinematicsFromPoE()
    {
        KDL::Frame H_S_T({-0.63401, 0, 0});
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, 0, 1}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, 1, 0}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1, 0, 0}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, 0, 1}, {-0.32901, 0, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1, 0, 0}, {-0.32901, 0, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, 0, 1}, {-0.54401, 0, 0}));

        return poe;
    }

    static KDL::Chain makeTeoRightLegKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        KDL::Chain chain;

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(        0,  KDL::PI_2,      0,         0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(        0,  KDL::PI_2,      0, KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    -0.33,          0,      0,         0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     -0.3,          0, 0.0175,         0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(        0, -KDL::PI_2,      0,         0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.123005,          0,      0,         0)));

        return chain;
    }

    static PoeExpression makeTeoRightLegKinematicsFromPoE()
    {
        KDL::Frame H_S_T(KDL::Rotation::RotY(-KDL::PI_2) * KDL::Rotation::RotX(KDL::PI_2), {0.0175, 0, -0.753005});
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0,  0, 1}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, -1, 0}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1,  0, 0}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1,  0, 0}, {     0, 0, -0.33}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1,  0, 0}, {0.0175, 0, -0.63}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, -1, 0}, {0.0175, 0, -0.63}));

        return poe;
    }

    static KDL::Chain makeAbbIrb120KinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotX(-KDL::PI_2))));

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(   0,  KDL::PI_2,  0.29,          0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.27,          0,     0,  KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.07,  KDL::PI_2,     0,          0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(   0, -KDL::PI_2, 0.302,          0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(   0,  KDL::PI_2,     0, -KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(   0,          0,  0.16,          0)));

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotZ(KDL::PI_2))));

        return chain;
    }

    static PoeExpression makeAbbIrb120KinematicsFromPoE()
    {
        KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI_2) * KDL::Rotation::RotZ(KDL::PI_2), {0.302, 0.47, 0});
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0,  1, 0}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0,  0, 1}, {    0, 0.29, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0,  0, 1}, {    0, 0.56, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1,  0, 0}, {0.302, 0.63, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0,  0, 1}, {0.302, 0.63, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, -1, 0}, {0.302, 0.63, 0}));

        return poe;
    }

    static KDL::Chain makePumaKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotX(-KDL::PI_2))));

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0,  KDL::PI_2, 2,  KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(1,          0, 0,  KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI_2, 0, -KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0,  KDL::PI_2, 2,          0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI_2, 0,  KDL::PI_2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0,          0, 1,          0)));

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotZ(-KDL::PI_2))));

        return chain;
    }

    static PoeExpression makePumaKinematicsFromPoE()
    {
        KDL::Frame H_S_T({0, 5, 1});
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, 1, 0}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1, 0, 0}, {0, 2, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1, 0, 0}, {0, 3, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, 1, 0}, {0, 3, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {1, 0, 0}, {0, 5, 0}));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, {0, 0, 1}, {0, 5, 0}));

        return poe;
    }

    static KDL::Chain makeStanfordKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        const KDL::Joint translZ(KDL::Joint::TransZ);

        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotX(-KDL::PI_2))));

        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0,  KDL::PI_2, 2,  KDL::PI_2)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0, -KDL::PI_2, 0,          0)));
        chain.addSegment(KDL::Segment(translZ, KDL::Frame::DH(0,          0, 0,          0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0,  KDL::PI_2, 3,          0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0, -KDL::PI_2, 0,  KDL::PI_2)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0,          0, 1,          0)));

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotZ(-KDL::PI_2))));

        return chain;
    }

    static PoeExpression makeStanfordKinematicsFromPoE()
    {
        KDL::Frame H_S_T({0, 5, 1});
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0, 1, 0}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {1, 0, 0}, {0, 2, 0}));
        poe.append(MatrixExponential(MatrixExponential::TRANSLATION, {0, 1, 0}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0, 1, 0}, {0, 2, 0}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {1, 0, 0}, {0, 5, 0}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0, 0, 1}, {0, 5, 0}));

        return poe;
    }

    static KDL::Chain makeAbbIrb910scKinematicsFromDH()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        const KDL::Joint translZ(KDL::Joint::TransZ);

        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotX(-KDL::PI_2))));

        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH( 0.4,       0,      0, 0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0.25, KDL::PI,      0, 0)));
        chain.addSegment(KDL::Segment(translZ, KDL::Frame::DH(   0,       0,      0, 0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(   0,       0, -0.125, 0)));

        return chain;
    }

    static PoeExpression makeAbbIrb910scKinematicsFromPoE()
    {
        KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI_2), {0.65, 0.125, 0});
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0,  1, 0}, KDL::Vector::Zero()));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0,  1, 0}, {0.4, 0, 0}));
        poe.append(MatrixExponential(MatrixExponential::TRANSLATION, {0, -1, 0}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0, -1, 0}, {0.65, 0, 0}));

        return poe;
    }

    static KDL::Chain makeAbbIrb6620lxFromDh()
    {
        const KDL::Joint rotZ(KDL::Joint::RotZ);
        const KDL::Joint translZ(KDL::Joint::TransZ);

        KDL::Chain chain;

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame({1.088, 2.5, 0})));

        chain.addSegment(KDL::Segment(translZ, KDL::Frame::DH( 0.38,          0,     0,         0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(0.975,          0,     0,         0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(  0.2,  KDL::PI_2,     0,         0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(    0, -KDL::PI_2, 0.887,         0)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(    0,  KDL::PI_2,     0, KDL::PI_2)));
        chain.addSegment(KDL::Segment(   rotZ, KDL::Frame::DH(    0,          0, 0.357,         0)));

        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RotZ(-KDL::PI_2))));

        return chain;
    }

    static PoeExpression makeAbbIrb6620lxFromPoE()
    {
        KDL::Frame H_S_T(KDL::Rotation::RotY(KDL::PI_2), {3, 1.613, 0});
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::TRANSLATION, {0,  0, 1}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0,  0, 1}, {1.468,   2.5, 0}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0,  0, 1}, {2.443,   2.5, 0}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0, -1, 0}, {2.643, 1.613, 0}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {0,  0, 1}, {2.643, 1.613, 0}));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, {1,  0, 0}, {2.643, 1.613, 0}));

        return poe;
    }

    static void checkSolutions(const ScrewTheoryIkSubproblem::Solutions & actual, const ScrewTheoryIkSubproblem::Solutions & expected)
    {
        ASSERT_EQ(actual.size(), expected.size());

        for (int i = 0; i < actual.size(); i++)
        {
            ASSERT_EQ(actual[i].size(), expected[i].size());

            for (int j = 0; j < actual[i].size(); j++)
            {
                ASSERT_NEAR(actual[i][j], expected[i][j], KDL::epsilon);
            }
        }
    }

    static void checkRobotKinematics(const KDL::Chain & chain, const PoeExpression & poe, int soln)
    {
        ASSERT_EQ(poe.size(), chain.getNrOfJoints());

        KDL::ChainFkSolverPos_recursive fkSolver(chain);
        KDL::Frame H_S_T_0_DH, H_S_T_0_ST;

        ASSERT_EQ(fkSolver.JntToCart(KDL::JntArray(chain.getNrOfJoints()), H_S_T_0_DH), KDL::SolverI::E_NOERROR);
        ASSERT_TRUE(poe.evaluate(KDL::JntArray(poe.size()), H_S_T_0_ST));
        ASSERT_EQ(H_S_T_0_ST, H_S_T_0_DH);

        // KDL::JntArray q = fillJointValues(chain.getNrOfJoints(), KDL::PI_2);
        KDL::JntArray q = fillJointValues(chain.getNrOfJoints(), 0);
        KDL::Frame H_S_T_q_DH, H_S_T_q_ST;

        ASSERT_EQ(fkSolver.JntToCart(q, H_S_T_q_DH), KDL::SolverI::E_NOERROR);
        ASSERT_TRUE(poe.evaluate(q, H_S_T_q_ST));
        ASSERT_EQ(H_S_T_q_ST, H_S_T_q_DH);

        ScrewTheoryIkProblemBuilder builder(poe);
        ScrewTheoryIkProblem * ikProblem = builder.build();

        ASSERT_TRUE(ikProblem);
        ASSERT_EQ(ikProblem->solutions(), soln);

        const auto & steps = ikProblem->getSteps();

        for (auto i = 0; i < steps.size(); i++)
        {
            const auto & step = steps[i];
            std::printf("[%d] %s [", i, step.second->describe());

            for (auto j = 0; j < step.first.size(); j++)
            {
                std::printf("%d%s", step.first[j], j < step.first.size() - 1 ? ", " : "");
            }

            std::printf("]\n");
        }

        std::cout << "qd: [";

        for (auto i = 0; i < q.rows(); i++)
        {
            std::cout << q(i) << (i < q.rows() - 1 ? ", " : "");
        }

        std::cout << "]" << std::endl;

        ScrewTheoryIkProblem::Solutions solutions;
        ASSERT_TRUE(ikProblem->solve(H_S_T_q_ST, q, solutions));
        delete ikProblem;

        bool match = false;

        for (auto j = 0; j < solutions.size(); j++)
        {
            const auto & solution = solutions[j];

            std::cout << "[" << j << "] [";

            for (auto i = 0; i < solution.rows(); i++)
            {
                std::cout << solution(i) << (i < solution.rows() - 1 ? ", " : "");
            }

            std::cout << "]" << std::endl;

            KDL::Frame H_S_T_q_ST_validate;
            ASSERT_TRUE(poe.evaluate(solution, H_S_T_q_ST_validate));
            ASSERT_EQ(H_S_T_q_ST_validate, H_S_T_q_ST);

            if (solution == q)
            {
                match = true;
            }
        }

        ASSERT_TRUE(match);
    }

    static int findTargetConfiguration(const ScrewTheoryIkProblem::Solutions & solutions, const KDL::JntArray & target)
    {
        for (int i = 0; i < solutions.size(); i++)
        {
            bool equal = true;

            for (int j = 0; j < solutions[i].rows(); j++)
            {
                if (!KDL::Equal(solutions[i](j), target(j)))
                {
                    equal = false;
                    break;
                }
            }

            if (equal)
            {
                return i;
            }
        }

        return -1;
    }
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

    KDL::Frame H_new_old(KDL::Rotation::RotZ(KDL::PI_2), {-1, 0, 0});
    MatrixExponential newExp = exp.cloneWithBase(H_new_old);

    ASSERT_EQ(newExp.getMotionType(), exp.getMotionType());
    ASSERT_EQ(newExp.getAxis(), KDL::Vector(0, 1, 0));
    ASSERT_EQ(newExp.getOrigin(), KDL::Vector(-2, 1, 1));

    exp.changeBase(H_new_old);

    ASSERT_EQ(exp.getMotionType(), newExp.getMotionType());
    ASSERT_EQ(exp.getAxis(), newExp.getAxis());
    ASSERT_EQ(exp.getOrigin(), newExp.getOrigin());
}

TEST_F(ScrewTheoryTest, MatrixExponentialRotation)
{
    double theta = KDL::PI_2;
    MatrixExponential exp(MatrixExponential::ROTATION, {0, 0, 1}, {1, 0, 0});

    KDL::Frame actual = exp.asFrame(theta);
    KDL::Frame expected(KDL::Rotation::RotZ(theta), {1, -1, 0});
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

TEST_F(ScrewTheoryTest, ProductOfExponentialsInit)
{
    PoeExpression poe;

    ASSERT_EQ(poe.size(), 0);
    ASSERT_EQ(poe.getTransform(), KDL::Frame::Identity());

    MatrixExponential exp(MatrixExponential::ROTATION, {1, 0, 0}, {1, 1, 1});
    poe.append(exp);

    ASSERT_EQ(poe.size(), 1);

    poe.changeBaseFrame(KDL::Frame(KDL::Rotation::RotZ(KDL::PI_2), {-1, 0, 0}));
    poe.changeToolFrame(KDL::Frame({0, 0, 1}));

    ASSERT_EQ(poe.getTransform(), KDL::Frame(KDL::Rotation::RotZ(KDL::PI_2), {-1, 0, 1}));

    KDL::Frame H;
    KDL::JntArray q(1);
    q(0) = KDL::PI_2;

    ASSERT_TRUE(poe.evaluate(q, H));
    ASSERT_EQ(H, KDL::Frame(KDL::Rotation(KDL::Rotation::RotZ(KDL::PI_2) * KDL::Rotation::RotX(KDL::PI_2)), {-2, 0, 0}));

    PoeExpression poe2(KDL::Frame(KDL::Rotation::RotX(KDL::PI_2), {0, 1, 0}));
    MatrixExponential exp2(MatrixExponential::ROTATION, {0, 1, 0}, {1, 0, 0});

    poe2.append(exp2);
    poe.append(poe2, KDL::Frame(KDL::Rotation::RotZ(KDL::PI_2), {0, 0, 1}));

    ASSERT_EQ(poe.size(), 2);
    ASSERT_EQ(poe.getTransform(), KDL::Frame(KDL::Rotation::RotZ(KDL::PI_2) * KDL::Rotation::RotX(KDL::PI_2), {-1, 0, 1}));

    q.resize(2);
    q(1) = -KDL::PI_2;

    ASSERT_TRUE(poe.evaluate(q, H));
    ASSERT_EQ(H, KDL::Frame(KDL::Rotation::RotX(KDL::PI), {-3, 1, 0}));
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

    KDL::JntArray q = fillJointValues(chain.getNrOfJoints(), KDL::PI_2);;
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

    KDL::JntArray q = fillJointValues(poe.size(), KDL::PI_2);;
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

    KDL::JntArray q = fillJointValues(poe.size(), KDL::PI_2);;
    KDL::Frame H_S_T_q_DH, H_S_T_q_ST;

    ASSERT_EQ(fkSolver.JntToCart(q, H_S_T_q_DH), KDL::SolverI::E_NOERROR);

    PoeExpression poeFromChain = PoeExpression::fromChain(chain);

    ASSERT_TRUE(poeFromChain.evaluate(q, H_S_T_q_ST));
    ASSERT_EQ(H_S_T_q_ST, H_S_T_q_DH);
}

TEST_F(ScrewTheoryTest, ProductOfExponentialsReverse)
{
    PoeExpression poe = makeTeoRightArmKinematicsFromPoE();
    PoeExpression poeReversed = poe.makeReverse();

    ASSERT_EQ(poeReversed.size(), poe.size());
    ASSERT_EQ(poeReversed.getTransform(), poe.getTransform().Inverse());

    KDL::Frame H_S_T_0, H_S_T_0_reversed;

    ASSERT_TRUE(poe.evaluate(KDL::JntArray(poe.size()), H_S_T_0));
    ASSERT_TRUE(poeReversed.evaluate(KDL::JntArray(poeReversed.size()), H_S_T_0_reversed));
    ASSERT_EQ(H_S_T_0_reversed, H_S_T_0.Inverse());

    KDL::JntArray q = fillJointValues(poe.size(), KDL::PI_2);;
    KDL::Frame H_S_T_q;

    ASSERT_TRUE(poe.evaluate(q, H_S_T_q));

    q = fillJointValues(poe.size(), -KDL::PI_2);

    KDL::Frame H_S_T_q_reversed;

    ASSERT_TRUE(poeReversed.evaluate(q, H_S_T_q_reversed));
    ASSERT_EQ(H_S_T_q_reversed, H_S_T_q.Inverse());

    poe.reverseSelf();

    ASSERT_TRUE(poe.evaluate(q, H_S_T_q_reversed));
    ASSERT_EQ(H_S_T_q_reversed, H_S_T_q.Inverse());
}

TEST_F(ScrewTheoryTest, PadenKahanOne)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(1, 1, 1);

    MatrixExponential exp(MatrixExponential::ROTATION, {0, 1, 0}, {1, 0, 0});
    PadenKahanOne pk1(exp, p);

    ASSERT_EQ(pk1.solutions(), 1);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pk1.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 1);

    ScrewTheoryIkSubproblem::Solutions expected = {{KDL::PI_2}};

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(0, 0, 1);
    KDL::Frame rhs2(k2 - p);
    ASSERT_FALSE(pk1.solve(rhs2, KDL::Frame::Identity(), actual));

    checkSolutions(actual, expected);

    KDL::Vector k3 = k + KDL::Vector(0, 1, 0);
    KDL::Frame rhs3(k3 - p);
    ASSERT_FALSE(pk1.solve(rhs3, KDL::Frame::Identity(), actual));

    checkSolutions(actual, expected);

    KDL::Vector k4 = k + KDL::Vector(0, 0, -1);
    KDL::Frame rhs4(k4 - p);
    ASSERT_FALSE(pk1.solve(rhs4, KDL::Frame::Identity(), actual));

    expected = {{0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}};
    ASSERT_FALSE(pk1.solve(rhs4, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector p2 = p + KDL::Vector(1, 0, 0);
    PadenKahanOne pk1b(exp, p2);
    KDL::Frame rhs5(k - p2);
    ASSERT_FALSE(pk1b.solve(rhs5, KDL::Frame::Identity(), actual));

    expected = {{0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}};
    ASSERT_FALSE(pk1b.solve(rhs5, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Frame rhs6(k4 - p2);
    ASSERT_TRUE(pk1b.solve(rhs6, KDL::Frame::Identity(), actual));

    expected = {{0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}};
    ASSERT_TRUE(pk1b.solve(rhs6, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector k7 = k4 + KDL::Vector(0, 1, 0);
    KDL::Frame rhs7(k7 - p2);
    ASSERT_FALSE(pk1b.solve(rhs7, KDL::Frame::Identity(), actual));

    expected = {{0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}};
    ASSERT_FALSE(pk1b.solve(rhs7, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PadenKahanTwo)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(1, -1, 1);
    KDL::Vector r(1, 0, 0);

    MatrixExponential exp1(MatrixExponential::ROTATION, {1, 0, 0}, r);
    MatrixExponential exp2(MatrixExponential::ROTATION, {0, 1, 0}, r);
    PadenKahanTwo pk2(exp1, exp2, p, r);

    ASSERT_EQ(pk2.solutions(), 2);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pk2.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 2);
    ASSERT_EQ(actual[1].size(), 2);

    ScrewTheoryIkSubproblem::Solutions expected = {
        {KDL::PI, -KDL::PI_2},
        {KDL::PI_2, KDL::PI_2}
    };

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(1, 0, 0);
    KDL::Frame rhs2(k2 - p);
    ASSERT_FALSE(pk2.solve(rhs2, KDL::Frame::Identity(), actual));

    expected = {
        {3 * KDL::PI_4, KDL::PI},
        {3 * KDL::PI_4, KDL::PI}
    };

    checkSolutions(actual, expected);

    KDL::Vector p3 = p + KDL::Vector(0.5, 0, 0);
    PadenKahanTwo pk2c(exp1, exp2, p3, r);
    KDL::Frame rhs3(k2 - p3);
    ASSERT_FALSE(pk2c.solve(rhs3, KDL::Frame::Identity(), actual));

    checkSolutions(actual, expected);

    KDL::Vector p4 = p + KDL::Vector(1, std::sqrt(2) - 1, 0);
    PadenKahanTwo pk2d(exp1, exp2, p4, r);
    KDL::Frame rhs4(k - p4);
    ASSERT_TRUE(pk2d.solve(rhs4, KDL::Frame::Identity(), actual));

    expected = {
        {3 * KDL::PI_4, 0},
        {3 * KDL::PI_4, 0}
    };

    checkSolutions(actual, expected);

    expected = {
        {3 * KDL::PI_4, KDL::PI},
        {3 * KDL::PI_4, KDL::PI}
    };

    ASSERT_TRUE(pk2d.solve(rhs4, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector p5 = p + KDL::Vector(1, 0, 0);
    PadenKahanTwo pk2e(exp1, exp2, p5, r);
    KDL::Frame rhs5(k - p5);
    ASSERT_FALSE(pk2e.solve(rhs5, KDL::Frame::Identity(), actual));

    expected = {
        {3 * KDL::PI_4, 0},
        {3 * KDL::PI_4, 0}
    };

    checkSolutions(actual, expected);

    expected = {
        {3 * KDL::PI_4, KDL::PI},
        {3 * KDL::PI_4, KDL::PI}
    };

    ASSERT_FALSE(pk2e.solve(rhs5, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector p6 = p + KDL::Vector(0, -1, 0);
    KDL::Vector k6 = k + KDL::Vector(1, 1, -1);
    PadenKahanTwo pk2f(exp1, exp2, p6, r);
    KDL::Frame rhs6(k6 - p6);
    ASSERT_TRUE(pk2f.solve(rhs6, KDL::Frame::Identity(), actual));

    expected = {
        {0, KDL::PI},
        {0, KDL::PI}
    };

    checkSolutions(actual, expected);

    expected = {
        {KDL::PI, KDL::PI},
        {KDL::PI, KDL::PI}
    };

    ASSERT_TRUE(pk2f.solve(rhs6, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector k7 = k + KDL::Vector(2, 1, -1);
    KDL::Frame rhs7(k7 - p6);
    ASSERT_FALSE(pk2f.solve(rhs7, KDL::Frame::Identity(), actual));

    expected = {
        {0, KDL::PI},
        {0, KDL::PI}
    };

    checkSolutions(actual, expected);

    expected = {
        {KDL::PI, KDL::PI},
        {KDL::PI, KDL::PI}
    };

    ASSERT_FALSE(pk2f.solve(rhs7, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector p8 = r;
    KDL::Vector k8 = r;
    PadenKahanTwo pk2g(exp1, exp2, p8, r);
    KDL::Frame rhs8(k8 - p8);
    ASSERT_TRUE(pk2g.solve(rhs8, KDL::Frame::Identity(), actual));

    expected = {
        {0, 0},
        {0, 0}
    };

    checkSolutions(actual, expected);

    expected = {
        {KDL::PI, KDL::PI},
        {KDL::PI, KDL::PI}
    };

    ASSERT_TRUE(pk2g.solve(rhs8, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector p9 = p + KDL::Vector(1, 0, 0);
    KDL::Vector k9 = k + KDL::Vector(1, 1, -1);
    PadenKahanTwo pk2h(exp1, exp2, p9, r);
    KDL::Frame rhs9(k9 - p9);
    ASSERT_FALSE(pk2h.solve(rhs9, KDL::Frame::Identity(), actual));

    expected = {
        {0, 0},
        {0, 0}
    };

    checkSolutions(actual, expected);

    expected = {
        {KDL::PI, KDL::PI},
        {KDL::PI, KDL::PI}
    };

    ASSERT_FALSE(pk2h.solve(rhs9, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PadenKahanThree)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(2, 1, 1);
    KDL::Vector delta(-1, 0, 0);

    MatrixExponential exp(MatrixExponential::ROTATION, {0, 1, 0}, {1, 0, 0});
    PadenKahanThree pk3(exp, p, k);

    ASSERT_EQ(pk3.solutions(), 2);

    KDL::Frame rhs(delta - (p - k));
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pk3.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 1);
    ASSERT_EQ(actual[1].size(), 1);

    ScrewTheoryIkSubproblem::Solutions expected = {
        {KDL::PI},
        {KDL::PI_2}
    };

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(1, 0, 1);
    PadenKahanThree pk3b(exp, p, k2);
    KDL::Frame rhs2(delta - (p - k2));

    ASSERT_FALSE(pk3b.solve(rhs2, KDL::Frame::Identity(), actual));

    expected = {
        {3 * KDL::PI_4},
        {3 * KDL::PI_4}
    };

    checkSolutions(actual, expected);

    KDL::Vector k3 = k + KDL::Vector(-1, 0, -1);
    PadenKahanThree pk3c(exp, p, k3);
    KDL::Frame rhs3(delta - (p - k3));

    ASSERT_TRUE(pk3c.solve(rhs3, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}, {KDL::PI}};
    ASSERT_TRUE(pk3c.solve(rhs3, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector delta4(-1, -1, 0);
    KDL::Frame rhs4(delta4 - (p - k3));

    ASSERT_FALSE(pk3c.solve(rhs4, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}, {KDL::PI}};
    ASSERT_FALSE(pk3c.solve(rhs4, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector p5 = p + KDL::Vector(1, 0, 0);
    PadenKahanThree pk3d(exp, p5, k);
    KDL::Frame rhs5(delta4 - (p5 - k));

    ASSERT_TRUE(pk3d.solve(rhs5, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}, {KDL::PI}};
    ASSERT_TRUE(pk3d.solve(rhs5, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Frame rhs6(delta - (p5 - k));
    ASSERT_FALSE(pk3d.solve(rhs6, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}, {KDL::PI}};
    ASSERT_FALSE(pk3d.solve(rhs6, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    PadenKahanThree pk3e(exp, p5, k3);
    KDL::Frame rhs7(KDL::Vector::Zero() - (p5 - k3));

    ASSERT_TRUE(pk3e.solve(rhs7, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}, {KDL::PI}};
    ASSERT_TRUE(pk3e.solve(rhs7, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Frame rhs8(delta - (p5 - k3));
    ASSERT_FALSE(pk3e.solve(rhs8, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}, {KDL::PI}};
    ASSERT_FALSE(pk3e.solve(rhs8, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector k9 = k3 + KDL::Vector(0, 1, 0);
    KDL::Vector delta9(0, -1, 0);
    KDL::Frame rhs9(delta9 - (p5 - k9));

    ASSERT_TRUE(pk3e.solve(rhs9, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}, {KDL::PI}};
    ASSERT_TRUE(pk3e.solve(rhs9, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Frame rhs10(delta - (p5 - k9));

    ASSERT_FALSE(pk3e.solve(rhs10, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};
    checkSolutions(actual, expected);

    expected = {{KDL::PI}, {KDL::PI}};
    ASSERT_FALSE(pk3e.solve(rhs10, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector p11 = p + KDL::Vector(-1, 0, 0);
    KDL::Vector k11 = k + KDL::Vector(0, 0, -1);
    KDL::Vector delta11(3, 0, 0);
    PadenKahanThree pk3f(exp, p11, k11);
    KDL::Frame rhs11(delta11 - (p11 - k11));

    ASSERT_TRUE(pk3f.solve(rhs11, KDL::Frame::Identity(), actual));

    expected = {{0}, {0}};

    checkSolutions(actual, expected);

    KDL::Vector k12 = k + KDL::Vector(1, 0, -1);
    PadenKahanThree pk3g(exp, p, k12);
    KDL::Frame rhs12(delta - (p - k12));

    ASSERT_TRUE(pk3g.solve(rhs12, KDL::Frame::Identity(), actual));

    expected = {{KDL::PI}, {KDL::PI}};

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosGotorOne)
{
    KDL::Vector p(1, 0, 0);
    KDL::Vector k(1, 1, 0);

    MatrixExponential exp(MatrixExponential::TRANSLATION, {0, 1, 0});
    PardosGotorOne pg1(exp, p);

    ASSERT_EQ(pg1.solutions(), 1);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pg1.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 1);

    ScrewTheoryIkSubproblem::Solutions expected = {{1.0}};

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosGotorTwo)
{
    KDL::Vector p(1, 1, 0);
    KDL::Vector k(2, 3, 0);

    MatrixExponential exp1(MatrixExponential::TRANSLATION, {0, 1, 0});
    MatrixExponential exp2(MatrixExponential::TRANSLATION, {1, 0, 0});
    PardosGotorTwo pg2(exp1, exp2, p);

    ASSERT_EQ(pg2.solutions(), 1);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pg2.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 2);

    ScrewTheoryIkSubproblem::Solutions expected = {{k.y() - p.y(), k.x() - p.x()}};

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosGotorThree)
{
    KDL::Vector p(1, 0, 0);
    KDL::Vector k(1, 2, 0);
    KDL::Vector delta(0, 1, 0);

    MatrixExponential exp(MatrixExponential::TRANSLATION, {0, 1, 0});
    PardosGotorThree pg3(exp, p, k);

    ASSERT_EQ(pg3.solutions(), 2);

    KDL::Frame rhs(delta - (p - k));
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pg3.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 1);
    ASSERT_EQ(actual[1].size(), 1);

    ScrewTheoryIkSubproblem::Solutions expected = {
        {k.y() - p.y() + delta.y()},
        {k.y() - p.y() - delta.y()}
    };

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(2, 0, 0);
    PardosGotorThree pg3b(exp, p, k2);
    KDL::Frame rhs2(delta - (p - k2));

    ASSERT_FALSE(pg3b.solve(rhs2, KDL::Frame::Identity(), actual));

    expected = {{2.0}, {2.0}};

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosGotorFour)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(3, 1, 1);

    MatrixExponential exp1(MatrixExponential::ROTATION, {0, 1, 0}, {2, 0, 0});
    MatrixExponential exp2(MatrixExponential::ROTATION, {0, 1, 0}, {1, 0, 0});
    PardosGotorFour pg4(exp1, exp2, p);

    ASSERT_EQ(pg4.solutions(), 2);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pg4.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 2);
    ASSERT_EQ(actual[1].size(), 2);

    ScrewTheoryIkSubproblem::Solutions expected = {
        {KDL::PI_2, KDL::PI_2},
        {KDL::PI, -KDL::PI_2}
    };

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(0, 1, 0);
    KDL::Frame rhs2(k2 - p);
    ASSERT_FALSE(pg4.solve(rhs2, KDL::Frame::Identity(), actual));

    checkSolutions(actual, expected);

    KDL::Vector p3 = p + KDL::Vector(0.75, 0, 0);
    KDL::Vector k3 = k + KDL::Vector(-0.75, 0, -0.75);
    PardosGotorFour pg4c(exp1, exp2, p3);
    KDL::Frame rhs3(k3 - p3);
    ASSERT_FALSE(pg4c.solve(rhs3, KDL::Frame::Identity(), actual));

    expected = {
        {3 * KDL::PI_4, KDL::PI},
        {3 * KDL::PI_4, KDL::PI}
    };

    checkSolutions(actual, expected);

    KDL::Vector k4 = k + KDL::Vector(-1, 0, -1);
    KDL::Frame rhs4(k4 - p);
    ASSERT_TRUE(pg4.solve(rhs4, KDL::Frame::Identity(), actual));

    expected = {
        {0, KDL::PI},
        {0, KDL::PI}
    };

    checkSolutions(actual, expected);

    expected = {
        {KDL::PI, KDL::PI},
        {KDL::PI, KDL::PI}
    };

    ASSERT_TRUE(pg4.solve(rhs4, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector k5 = k4 + KDL::Vector(0, 1, 0);
    KDL::Frame rhs5(k5 - p);
    ASSERT_FALSE(pg4.solve(rhs5, KDL::Frame::Identity(), actual));

    expected = {
        {0, KDL::PI},
        {0, KDL::PI}
    };

    checkSolutions(actual, expected);

    expected = {
        {KDL::PI, KDL::PI},
        {KDL::PI, KDL::PI}
    };

    ASSERT_FALSE(pg4.solve(rhs5, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Vector p6 = p + KDL::Vector(1, 0, 0);
    KDL::Vector k6 = k + KDL::Vector(-1, 0, 0);
    PardosGotorFour pg4d(exp1, exp2, p6);
    KDL::Frame rhs6(k6 - p6);
    ASSERT_TRUE(pg4d.solve(rhs6, KDL::Frame::Identity(), actual));

    expected = {
        {KDL::PI_2, 0},
        {KDL::PI_2, 0}
    };

    checkSolutions(actual, expected);

    expected = {
        {KDL::PI_2, KDL::PI},
        {KDL::PI_2, KDL::PI}
    };

    ASSERT_TRUE(pg4d.solve(rhs6, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Frame rhs7(k - p6);
    ASSERT_FALSE(pg4d.solve(rhs7, KDL::Frame::Identity(), actual));

    expected = {
        {3 * KDL::PI_4, 0},
        {3 * KDL::PI_4, 0}
    };

    checkSolutions(actual, expected);

    expected = {
        {3 * KDL::PI_4, KDL::PI},
        {3 * KDL::PI_4, KDL::PI}
    };

    ASSERT_FALSE(pg4d.solve(rhs7, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);

    KDL::Frame rhs8(k4 - p6);
    ASSERT_FALSE(pg4d.solve(rhs8, KDL::Frame::Identity(), actual));

    expected = {
        {0, 0},
        {0, 0}
    };

    checkSolutions(actual, expected);

    expected = {
        {KDL::PI, KDL::PI},
        {KDL::PI, KDL::PI}
    };

    ASSERT_FALSE(pg4d.solve(rhs8, KDL::Frame::Identity(), expected[0], actual));

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, AbbIrb120Kinematics)
{
    KDL::Chain chain = makeAbbIrb120KinematicsFromDH();
    PoeExpression poe = makeAbbIrb120KinematicsFromPoE();

    checkRobotKinematics(chain, poe, 8);
}

TEST_F(ScrewTheoryTest, PumaKinematics)
{
    KDL::Chain chain = makePumaKinematicsFromDH();
    PoeExpression poe = makePumaKinematicsFromPoE();

    checkRobotKinematics(chain, poe, 8);
}

TEST_F(ScrewTheoryTest, StanfordKinematics)
{
    KDL::Chain chain = makeStanfordKinematicsFromDH();
    PoeExpression poe = makeStanfordKinematicsFromPoE();

    checkRobotKinematics(chain, poe, 8);
}

TEST_F(ScrewTheoryTest, AbbIrb910scKinematics)
{
    KDL::Chain chain = makeAbbIrb910scKinematicsFromDH();
    PoeExpression poe = makeAbbIrb910scKinematicsFromPoE();

    checkRobotKinematics(chain, poe, 2);
}

TEST_F(ScrewTheoryTest, AbbIrb6620lxKinematics)
{
    KDL::Chain chain = makeAbbIrb6620lxFromDh();
    PoeExpression poe = makeAbbIrb6620lxFromPoE();

    checkRobotKinematics(chain, poe, 4);
}

TEST_F(ScrewTheoryTest, TeoRightArmKinematics)
{
    KDL::Chain chain = makeTeoRightArmKinematicsFromDH();
    PoeExpression poe = makeTeoRightArmKinematicsFromPoE();

    checkRobotKinematics(chain, poe, 8);
}

TEST_F(ScrewTheoryTest, TeoRightLegKinematics)
{
    KDL::Chain chain = makeTeoRightLegKinematicsFromDH();
    PoeExpression poe = makeTeoRightLegKinematicsFromPoE();

    checkRobotKinematics(chain, poe, 8);
}

TEST_F(ScrewTheoryTest, ConfigurationSelector)
{
    PoeExpression poe = makeTeoRightArmKinematicsFromPoE();

    KDL::JntArray q(poe.size());
    q(3) = KDL::PI_2; // elbow

    KDL::Frame H;
    ASSERT_TRUE(poe.evaluate(q, H));

    ScrewTheoryIkProblemBuilder builder(poe);
    ScrewTheoryIkProblem * ikProblem = builder.build();

    ASSERT_TRUE(ikProblem);
    ASSERT_EQ(ikProblem->solutions(), 8);

    ScrewTheoryIkProblem::Solutions solutions;
    ASSERT_TRUE(ikProblem->solve(H, q, solutions));

    KDL::JntArray qMin = fillJointValues(poe.size(), -KDL::PI);
    KDL::JntArray qMax = fillJointValues(poe.size(), KDL::PI);

    ConfigurationSelectorLeastOverallAngularDisplacementFactory confFactory(qMin, qMax);
    ConfigurationSelector * config = confFactory.create();

    ASSERT_TRUE(config);
    ASSERT_TRUE(config->configure(solutions));
    ASSERT_TRUE(config->findOptimalConfiguration(q));

    KDL::JntArray qSolved;
    config->retrievePose(qSolved);
    int n1 = findTargetConfiguration(solutions, q);

    ASSERT_NE(n1, -1);

    H.p += {0.01, 0, 0}; // add a tiny displacement

    ASSERT_TRUE(ikProblem->solve(H, q, solutions));
    delete ikProblem;

    ASSERT_TRUE(config->configure(solutions));
    ASSERT_TRUE(config->findOptimalConfiguration(q));

    config->retrievePose(qSolved);
    int n2 = findTargetConfiguration(solutions, qSolved);

    ASSERT_EQ(n2, n1);
    delete config;
}

TEST_F(ScrewTheoryTest, ConfigurationSelectorGait)
{
    PoeExpression poe = makeTeoRightLegKinematicsFromPoE();

    KDL::JntArray q(poe.size());
    q(2) = -0.3; // approx. 20 degrees
    q(3) = 0.6;
    q(4) = -0.3;

    KDL::Frame H;
    ASSERT_TRUE(poe.evaluate(q, H));

    ScrewTheoryIkProblemBuilder builder(poe);
    ScrewTheoryIkProblem * ikProblem = builder.build();

    ASSERT_TRUE(ikProblem);
    ASSERT_EQ(ikProblem->solutions(), 8);

    ScrewTheoryIkProblem::Solutions solutions;
    ASSERT_TRUE(ikProblem->solve(H, q, solutions));

    KDL::JntArray qMin = fillJointValues(poe.size(), -KDL::PI);
    KDL::JntArray qMax = fillJointValues(poe.size(), KDL::PI);

    ConfigurationSelectorHumanoidGaitFactory confFactory(qMin, qMax);
    ConfigurationSelector * config = confFactory.create();

    KDL::JntArray qInitial(poe.size());

    ASSERT_TRUE(config);
    ASSERT_TRUE(config->configure(solutions));
    ASSERT_TRUE(config->findOptimalConfiguration(qInitial));

    KDL::JntArray qSolved;
    config->retrievePose(qSolved);
    int n1 = findTargetConfiguration(solutions, q);

    ASSERT_NE(n1, -1);
    delete config;
}

} // namespace roboticslab::test

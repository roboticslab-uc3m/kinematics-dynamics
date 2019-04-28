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

#include "ConfigurationSelector.hpp"
#include "MatrixExponential.hpp"
#include "ProductOfExponentials.hpp"
#include "ScrewTheoryIkProblem.hpp"
#include "ScrewTheoryIkSubproblems.hpp"

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-tests
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
        KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI / 2) * KDL::Rotation::RotZ(KDL::PI), KDL::Vector(-0.718506, 0, 0));
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector( 0,  0, 1), KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector( 0, -1, 0), KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(-1,  0, 0), KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector( 0,  0, 1), KDL::Vector(-0.32901, 0, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(-1,  0, 0), KDL::Vector(-0.32901, 0, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector( 0,  0, 1), KDL::Vector(-0.53101, 0, 0)));

        return poe;
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
        KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI / 2) * KDL::Rotation::RotZ(KDL::PI / 2), KDL::Vector(0.302, 0.47, 0));
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0,  1, 0), KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(    0, 0.29, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(    0, 0.56, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(1,  0, 0), KDL::Vector(0.302, 0.63, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(0.302, 0.63, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, -1, 0), KDL::Vector(0.302, 0.63, 0)));

        return poe;
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
        KDL::Frame H_S_T(KDL::Vector(0, 5, 1));
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector::Zero()));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 2, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 3, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(0, 3, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 5, 0)));
        poe.append(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 0, 1), KDL::Vector(0, 5, 0)));

        return poe;
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
        KDL::Frame H_S_T(KDL::Vector(0, 5, 1));
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector::Zero()));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 2, 0)));
        poe.append(MatrixExponential(MatrixExponential::TRANSLATION, KDL::Vector(0, 1, 0)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(0, 2, 0)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(0, 5, 0)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, 0, 1), KDL::Vector(0, 5, 0)));

        return poe;
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
        KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI / 2), KDL::Vector(0.65, 0.125, 0));
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  1, 0), KDL::Vector::Zero()));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  1, 0), KDL::Vector(0.4, 0, 0)));
        poe.append(MatrixExponential(MatrixExponential::TRANSLATION, KDL::Vector(0, -1, 0)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, -1, 0), KDL::Vector(0.65, 0, 0)));

        return poe;
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
        KDL::Frame H_S_T(KDL::Rotation::RotY(KDL::PI / 2), KDL::Vector(3, 1.613, 0));
        PoeExpression poe(H_S_T);

        poe.append(MatrixExponential(MatrixExponential::TRANSLATION, KDL::Vector(0,  0, 1)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(1.468,   2.5, 0)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(2.443,   2.5, 0)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0, -1, 0), KDL::Vector(2.643, 1.613, 0)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(2.643, 1.613, 0)));
        poe.append(MatrixExponential(   MatrixExponential::ROTATION, KDL::Vector(1,  0, 0), KDL::Vector(2.643, 1.613, 0)));

        return poe;
    }

    static void checkSolutions(const ScrewTheoryIkSubproblem::Solutions & actual, const ScrewTheoryIkSubproblem::Solutions & expected)
    {
        ScrewTheoryIkSubproblem::JointIdsToSolutions actualSorted, expectedSorted;

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

    static void checkRobotKinematics(const KDL::Chain & chain, const PoeExpression & poe, int soln)
    {
        ASSERT_EQ(poe.size(), chain.getNrOfJoints());

        KDL::ChainFkSolverPos_recursive fkSolver(chain);
        KDL::Frame H_S_T_0_DH, H_S_T_0_ST;

        ASSERT_EQ(fkSolver.JntToCart(KDL::JntArray(chain.getNrOfJoints()), H_S_T_0_DH), KDL::SolverI::E_NOERROR);
        ASSERT_TRUE(poe.evaluate(KDL::JntArray(poe.size()), H_S_T_0_ST));
        ASSERT_EQ(H_S_T_0_ST, H_S_T_0_DH);

        KDL::JntArray q = fillJointValues(chain.getNrOfJoints(), KDL::PI / 2);
        KDL::Frame H_S_T_q_DH, H_S_T_q_ST;

        ASSERT_EQ(fkSolver.JntToCart(q, H_S_T_q_DH), KDL::SolverI::E_NOERROR);
        ASSERT_TRUE(poe.evaluate(q, H_S_T_q_ST));
        ASSERT_EQ(H_S_T_q_ST, H_S_T_q_DH);

        ScrewTheoryIkProblemBuilder builder(poe);
        ScrewTheoryIkProblem * ikProblem = builder.build();

        ASSERT_TRUE(ikProblem);
        ASSERT_EQ(ikProblem->solutions(), soln);

        ScrewTheoryIkProblem::Solutions solutions;
        ASSERT_TRUE(ikProblem->solve(H_S_T_q_ST, solutions));
        delete ikProblem;

        for (int i = 0; i < solutions.size(); i++)
        {
            KDL::Frame H_S_T_q_ST_validate;
            ASSERT_TRUE(poe.evaluate(solutions[i], H_S_T_q_ST_validate));
            ASSERT_EQ(H_S_T_q_ST_validate, H_S_T_q_ST);
        }
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

    KDL::Frame H_new_old(KDL::Rotation::RotZ(KDL::PI / 2), KDL::Vector(-1, 0, 0));
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

TEST_F(ScrewTheoryTest, ProductOfExponentialsInit)
{
    PoeExpression poe;

    ASSERT_EQ(poe.size(), 0);
    ASSERT_EQ(poe.getTransform(), KDL::Frame::Identity());

    MatrixExponential exp(MatrixExponential::ROTATION, KDL::Vector(1, 0, 0), KDL::Vector(1, 1, 1));
    poe.append(exp);

    ASSERT_EQ(poe.size(), 1);

    poe.changeBaseFrame(KDL::Frame(KDL::Rotation::RotZ(KDL::PI / 2), KDL::Vector(-1, 0, 0)));
    poe.changeToolFrame(KDL::Frame(KDL::Vector(0, 0, 1)));

    ASSERT_EQ(poe.getTransform(), KDL::Frame(KDL::Rotation::RotZ(KDL::PI / 2), KDL::Vector(-1, 0, 1)));

    KDL::Frame H;
    KDL::JntArray q(1);
    q(0) = KDL::PI / 2;

    ASSERT_TRUE(poe.evaluate(q, H));
    ASSERT_EQ(H, KDL::Frame(KDL::Rotation(KDL::Rotation::RotZ(KDL::PI / 2) * KDL::Rotation::RotX(KDL::PI / 2)), KDL::Vector(-2, 0, 0)));

    PoeExpression poe2(KDL::Frame(KDL::Rotation::RotX(KDL::PI / 2), KDL::Vector(0, 1, 0)));
    MatrixExponential exp2(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(1, 0, 0));

    poe2.append(exp2);
    poe.append(poe2, KDL::Frame(KDL::Rotation::RotZ(KDL::PI / 2), KDL::Vector(0, 0, 1)));

    ASSERT_EQ(poe.size(), 2);
    ASSERT_EQ(poe.getTransform(), KDL::Frame(KDL::Rotation::RotZ(KDL::PI / 2) * KDL::Rotation::RotX(KDL::PI / 2), KDL::Vector(-1, 0, 1)));

    q.resize(2);
    q(1) = -KDL::PI / 2;

    ASSERT_TRUE(poe.evaluate(q, H));
    ASSERT_EQ(H, KDL::Frame(KDL::Rotation::RotX(KDL::PI), KDL::Vector(-3, 1, 0)));
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

    KDL::JntArray q = fillJointValues(chain.getNrOfJoints(), KDL::PI / 2);;
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

    KDL::JntArray q = fillJointValues(poe.size(), KDL::PI / 2);;
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

    KDL::JntArray q = fillJointValues(poe.size(), KDL::PI / 2);;
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

    KDL::JntArray q = fillJointValues(poe.size(), KDL::PI / 2);;
    KDL::Frame H_S_T_q;

    ASSERT_TRUE(poe.evaluate(q, H_S_T_q));

    q = fillJointValues(poe.size(), -KDL::PI / 2);

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

    MatrixExponential exp(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(1, 0, 0));
    PadenKahanOne pk1(0, exp, p);

    ASSERT_EQ(pk1.solutions(), 1);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pk1.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 1);

    ScrewTheoryIkSubproblem::Solutions expected(1);
    ScrewTheoryIkSubproblem::JointIdsToSolutions sol(1);

    sol[0] = std::make_pair(0, KDL::PI / 2);
    expected[0] = sol;

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(0, 0, 1);
    KDL::Frame rhs2(k2 - p);
    ASSERT_FALSE(pk1.solve(rhs2, KDL::Frame::Identity(), actual));

    checkSolutions(actual, expected);

    KDL::Vector k3 = k + KDL::Vector(0, 1, 0);
    KDL::Frame rhs3(k3 - p);
    ASSERT_FALSE(pk1.solve(rhs3, KDL::Frame::Identity(), actual));

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

    ASSERT_EQ(pk2.solutions(), 2);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pk2.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 2);
    ASSERT_EQ(actual[1].size(), 2);

    ScrewTheoryIkSubproblem::Solutions expected(2);
    ScrewTheoryIkSubproblem::JointIdsToSolutions sols1(2), sols2(2);

    sols1[0] = std::make_pair(0, KDL::PI / 2);
    sols1[1] = std::make_pair(1, KDL::PI / 2);

    sols2[0] = std::make_pair(0, KDL::PI);
    sols2[1] = std::make_pair(1, -KDL::PI / 2);

    expected[0] = sols1;
    expected[1] = sols2;

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(1, 0, 0);
    KDL::Frame rhs2(k2 - p);
    ASSERT_FALSE(pk2.solve(rhs2, KDL::Frame::Identity(), actual));

    sols1[0].second = sols2[0].second = 3 * KDL::PI / 4;
    sols1[1].second = sols2[1].second = KDL::PI;

    expected[0] = sols1;
    expected[1] = sols2;

    checkSolutions(actual, expected);

    KDL::Vector p3 = p + KDL::Vector(0.5, 0, 0);
    PadenKahanTwo pk2c(0, 1, exp1, exp2, p3, r);
    KDL::Frame rhs3(k2 - p3);
    ASSERT_FALSE(pk2c.solve(rhs3, KDL::Frame::Identity(), actual));

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PadenKahanThree)
{
    KDL::Vector p(0, 1, 0);
    KDL::Vector k(2, 1, 1);
    KDL::Vector delta(-1, 0, 0);

    MatrixExponential exp(MatrixExponential::ROTATION, KDL::Vector(0, 1, 0), KDL::Vector(1, 0, 0));
    PadenKahanThree pk3(0, exp, p, k);

    ASSERT_EQ(pk3.solutions(), 2);

    KDL::Frame rhs(delta - (p - k));
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pk3.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 1);
    ASSERT_EQ(actual[1].size(), 1);

    ScrewTheoryIkSubproblem::Solutions expected(2);
    ScrewTheoryIkSubproblem::JointIdsToSolutions sol1(1), sol2(1);

    sol1[0] = std::make_pair(0, KDL::PI / 2);
    sol2[0] = std::make_pair(0, KDL::PI);

    expected[0] = sol1;
    expected[1] = sol2;

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(1, 0, 1);
    PadenKahanThree pk3b(0, exp, p, k2);
    KDL::Frame rhs2(delta - (p - k2));

    ASSERT_FALSE(pk3b.solve(rhs2, KDL::Frame::Identity(), actual));

    sol1[0] = sol2[0] = std::make_pair(0, 3 * KDL::PI / 4);

    expected[0] = sol1;
    expected[1] = sol2;

    checkSolutions(actual, expected);
}

TEST_F(ScrewTheoryTest, PardosOne)
{
    KDL::Vector p(1, 0, 0);
    KDL::Vector k(1, 1, 0);

    MatrixExponential exp(MatrixExponential::TRANSLATION, KDL::Vector(0, 1, 0));
    PardosGotorOne pg1(0, exp, p);

    ASSERT_EQ(pg1.solutions(), 1);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pg1.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 1);

    ScrewTheoryIkSubproblem::Solutions expected(1);
    ScrewTheoryIkSubproblem::JointIdsToSolutions sol(1);

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
    PardosGotorTwo pg2(0, 1, exp1, exp2, p);

    ASSERT_EQ(pg2.solutions(), 1);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pg2.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 1);
    ASSERT_EQ(actual[0].size(), 2);

    ScrewTheoryIkSubproblem::Solutions expected(1);
    ScrewTheoryIkSubproblem::JointIdsToSolutions sols(2);

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
    PardosGotorThree pg3(0, exp, p, k);

    ASSERT_EQ(pg3.solutions(), 2);

    KDL::Frame rhs(delta - (p - k));
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pg3.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 1);
    ASSERT_EQ(actual[1].size(), 1);

    ScrewTheoryIkSubproblem::Solutions expected(2);
    ScrewTheoryIkSubproblem::JointIdsToSolutions sol1(1), sol2(1);

    sol1[0] = std::make_pair(0, k.y() - p.y() - delta.y());
    sol2[0] = std::make_pair(0, k.y() - p.y() + delta.y());

    expected[0] = sol1;
    expected[1] = sol2;

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(2, 0, 0);
    PardosGotorThree pg3b(0, exp, p, k2);
    KDL::Frame rhs2(delta - (p - k2));

    ASSERT_FALSE(pg3b.solve(rhs2, KDL::Frame::Identity(), actual));

    sol1[0] = sol2[0] = std::make_pair(0, 2);

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
    PardosGotorFour pg4(0, 1, exp1, exp2, p);

    ASSERT_EQ(pg4.solutions(), 2);

    KDL::Frame rhs(k - p);
    ScrewTheoryIkSubproblem::Solutions actual;
    ASSERT_TRUE(pg4.solve(rhs, KDL::Frame::Identity(), actual));

    ASSERT_EQ(actual.size(), 2);
    ASSERT_EQ(actual[0].size(), 2);
    ASSERT_EQ(actual[1].size(), 2);

    ScrewTheoryIkSubproblem::Solutions expected(2);
    ScrewTheoryIkSubproblem::JointIdsToSolutions sols1(2), sols2(2);

    sols1[0] = std::make_pair(0, KDL::PI / 2);
    sols1[1] = std::make_pair(1, KDL::PI / 2);

    sols2[0] = std::make_pair(0, KDL::PI);
    sols2[1] = std::make_pair(1, -KDL::PI / 2);

    expected[0] = sols1;
    expected[1] = sols2;

    checkSolutions(actual, expected);

    KDL::Vector k2 = k + KDL::Vector(0, 1, 0);
    KDL::Frame rhs2(k2 - p);
    ASSERT_FALSE(pg4.solve(rhs2, KDL::Frame::Identity(), actual));

    checkSolutions(actual, expected);

    KDL::Vector p3 = p + KDL::Vector(0.75, 0, 0);
    KDL::Vector k3 = k + KDL::Vector(-0.75, 0, -0.75);
    PardosGotorFour pg4c(0, 1, exp1, exp2, p3);
    KDL::Frame rhs3(k3 - p3);
    ASSERT_FALSE(pg4c.solve(rhs3, KDL::Frame::Identity(), actual));

    sols1[0].second = sols2[0].second = 3 * KDL::PI / 4;
    sols1[1].second = sols2[1].second = KDL::PI;

    expected[0] = sols1;
    expected[1] = sols2;

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

TEST_F(ScrewTheoryTest, ConfigurationSelector)
{
    PoeExpression poe = makeTeoRightArmKinematicsFromPoE();

    KDL::JntArray q(poe.size());
    q(3) = KDL::PI / 2; // elbow

    KDL::Frame H;
    ASSERT_TRUE(poe.evaluate(q, H));

    ScrewTheoryIkProblemBuilder builder(poe);
    ScrewTheoryIkProblem * ikProblem = builder.build();

    ASSERT_TRUE(ikProblem);
    ASSERT_EQ(ikProblem->solutions(), 8);

    ScrewTheoryIkProblem::Solutions solutions;
    ASSERT_TRUE(ikProblem->solve(H, solutions));

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

    H.p += KDL::Vector(0.01, 0, 0); // add a tiny displacement

    ASSERT_TRUE(ikProblem->solve(H, solutions));
    delete ikProblem;

    ASSERT_TRUE(config->configure(solutions));
    ASSERT_TRUE(config->findOptimalConfiguration(q));

    config->retrievePose(qSolved);
    int n2 = findTargetConfiguration(solutions, qSolved);

    ASSERT_EQ(n2, n1);
    delete config;
}

}  // namespace roboticslab

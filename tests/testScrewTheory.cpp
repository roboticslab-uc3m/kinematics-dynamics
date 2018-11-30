#include "gtest/gtest.h"

#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/utilities/utility.h>

#include "MatrixExponential.hpp"
#include "ProductOfExponentials.hpp"

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

    static KDL::Chain makeTeoRightArmKinematics()
    {
        KDL::Joint rotZ(KDL::Joint::RotZ);
        KDL::Chain chain;

        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0,  KDL::PI / 2,     0.0,          0.0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0,  KDL::PI / 2,     0.0, -KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0,  KDL::PI / 2, 0.32901, -KDL::PI / 2)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0, -KDL::PI / 2,     0.0,          0.0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(     0.0,  KDL::PI / 2,   0.202,          0.0)));
        chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.187496, -KDL::PI / 2,     0.0,  KDL::PI / 2)));

        return chain;
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
    KDL::Chain chain = makeTeoRightArmKinematics();
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
    std::vector<MatrixExponential> exps;

    exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 0, 1), KDL::Vector::Zero()));
    exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, -1, 0), KDL::Vector::Zero()));
    exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(-1, 0, 0), KDL::Vector::Zero()));
    exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 0, 1), KDL::Vector(-0.32901, 0, 0)));
    exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(-1, 0, 0), KDL::Vector(-0.32901, 0, 0)));
    exps.push_back(MatrixExponential(MatrixExponential::ROTATION, KDL::Vector(0, 0, 1), KDL::Vector(-0.53101, 0, 0)));

    KDL::Frame H_S_T(KDL::Rotation::RotX(KDL::PI / 2) * KDL::Rotation::RotZ(KDL::PI), KDL::Vector(-0.718506, 0, 0));

    PoeExpression poe(exps, H_S_T);

    ASSERT_EQ(poe.size(), exps.size());
    ASSERT_EQ(poe.getTransform(), H_S_T);

    MatrixExponential exp3 = poe.exponentialAtJoint(3);
    ASSERT_EQ(exp3.getMotionType(), exps[3].getMotionType());
    ASSERT_EQ(exp3.getAxis(), exps[3].getAxis());
    ASSERT_EQ(exp3.getOrigin(), exps[3].getOrigin());
    ASSERT_EQ(exp3.asFrame(KDL::PI / 2), exps[3].asFrame(KDL::PI / 2));

    KDL::Chain chainDH = makeTeoRightArmKinematics();
    KDL::Chain chainST = poe.toChain();

    KDL::ChainFkSolverPos_recursive fkSolverDH(chainDH);
    KDL::ChainFkSolverPos_recursive fkSolverST(chainST);

    KDL::JntArray q(exps.size());

    for (int i = 0; i < exps.size(); i++)
    {
        q(i) = KDL::PI / 2;
    }

    KDL::Frame H_S_T_q_DH, H_S_T_q_ST;
    ASSERT_EQ(fkSolverDH.JntToCart(q, H_S_T_q_DH), KDL::SolverI::E_NOERROR);
    ASSERT_EQ(fkSolverST.JntToCart(q, H_S_T_q_ST), KDL::SolverI::E_NOERROR);
    ASSERT_EQ(H_S_T_q_ST, H_S_T_q_DH);
}

}  // namespace roboticslab

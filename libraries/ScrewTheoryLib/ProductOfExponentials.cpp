// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProductOfExponentials.hpp"

#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    const int UNKNOWN_OR_STATIC_JOINT = -1;

    inline KDL::Joint::JointType motionToJointType(MatrixExponential::motion motionType)
    {
        switch (motionType)
        {
        case MatrixExponential::ROTATION:
            return KDL::Joint::RotAxis;
        case MatrixExponential::TRANSLATION:
            return KDL::Joint::TransAxis;
        default:
            return KDL::Joint::None;
        }
    }

    inline int jointTypeToMotionId(KDL::Joint::JointType jointType)
    {
        switch (jointType)
        {
        case KDL::Joint::RotAxis:
        case KDL::Joint::RotX:
        case KDL::Joint::RotY:
        case KDL::Joint::RotZ:
            return MatrixExponential::ROTATION;
        case KDL::Joint::TransAxis:
        case KDL::Joint::TransX:
        case KDL::Joint::TransY:
        case KDL::Joint::TransZ:
            return MatrixExponential::TRANSLATION;
        case KDL::Joint::None:
        default:
            return UNKNOWN_OR_STATIC_JOINT;
        }
    }
}

// -----------------------------------------------------------------------------

PoeExpression::PoeExpression(const std::vector<MatrixExponential> & _exps, const KDL::Frame & _H_ST)
    : exps(_exps),
      H_S_T(_H_ST)
{}

// -----------------------------------------------------------------------------

PoeExpression::PoeExpression()
{}

// -----------------------------------------------------------------------------

bool PoeExpression::evaluate(const KDL::JntArray & q, KDL::Frame & H) const
{
    if (exps.size() != q.rows())
    {
        CD_WARNING("Size mismatch: %d (terms of PoE) != %d (joint array).\n", exps.size(), q.rows());
        return false;
    }

    H = KDL::Frame::Identity();

    for (int i = 0; i < exps.size(); i++)
    {
        H = H * exps[i].asFrame(q(i));
    }

    H = H * H_S_T;

    return true;
}

// -----------------------------------------------------------------------------

PoeExpression PoeExpression::reverse() const
{
    PoeExpression poeReversed;

    poeReversed.exps.reserve(exps.size());
    poeReversed.H_S_T = H_S_T.Inverse();

    for (int i = exps.size() - 1; i >= 0; i--)
    {
        const MatrixExponential & exp = exps[i];

        KDL::Vector newAxis = poeReversed.H_S_T.M * exp.getAxis();
        KDL::Vector newOrigin = KDL::Vector::Zero();

        if (exp.getMotionType() == MatrixExponential::ROTATION)
        {
            newOrigin = poeReversed.H_S_T * exp.getOrigin();
        }

        MatrixExponential newExp(MatrixExponential(exp.getMotionType(), newAxis, newOrigin));
        poeReversed.exps.push_back(newExp);
    }

    return poeReversed;
}

// -----------------------------------------------------------------------------

KDL::Chain PoeExpression::toChain() const
{
    KDL::Chain chain;
    KDL::Frame H_S_prev = KDL::Frame::Identity();

    for (int i = 0; i < exps.size(); i++)
    {
        const MatrixExponential & exp = exps[i];

        KDL::Joint::JointType jointType = motionToJointType(exp.getMotionType());
        KDL::Joint joint(H_S_prev.Inverse() * exp.getOrigin(), exp.getAxis(), jointType);

        // http://www.orocos.org/wiki/main-page/kdl-wiki/user-manual/kinematic-trees
        KDL::Segment segment(joint, KDL::Frame(joint.JointOrigin()));
        chain.addSegment(segment);

        // update position, keep orientation
        H_S_prev = KDL::Frame(exp.getOrigin());
    }

    KDL::Frame H_N_T = H_S_prev.Inverse() * H_S_T;
    KDL::Segment toolSegment(KDL::Joint(KDL::Joint::None), H_N_T);
    chain.addSegment(toolSegment);

    return chain;
}

// -----------------------------------------------------------------------------

PoeExpression PoeExpression::fromChain(const KDL::Chain & chain)
{
    PoeExpression poe;
    KDL::Frame H_S_prev = KDL::Frame::Identity();

    poe.exps.reserve(chain.getNrOfJoints());

    for (int i = 0; i < chain.getNrOfSegments(); i++)
    {
        const KDL::Segment & segment = chain.getSegment(i);
        const KDL::Joint & joint = segment.getJoint();
        int motionTypeId = jointTypeToMotionId(joint.getType());

        if (motionTypeId != UNKNOWN_OR_STATIC_JOINT)
        {
            MatrixExponential::motion motionType = static_cast<MatrixExponential::motion>(motionTypeId);
            MatrixExponential exp(motionType, H_S_prev.M * joint.JointAxis(), H_S_prev * joint.JointOrigin());
            poe.exps.push_back(exp);
        }

        H_S_prev = H_S_prev * segment.pose(0);
    }

    poe.H_S_T = H_S_prev;

    return poe;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ChainIkSolverPos_ID.hpp"

#include <limits>

#include <kdl/frames.hpp>

using namespace roboticslab;

// -----------------------------------------------------------------------------

ChainIkSolverPos_ID::ChainIkSolverPos_ID(const KDL::Chain & _chain, const KDL::JntArray & q_min,
        const KDL::JntArray & q_max, KDL::ChainFkSolverPos & fksolver, KDL::ChainIkSolverVel & iksolver)
    : chain(_chain),
      nj(chain.getNrOfJoints()),
      qMin(nj),
      qMax(nj),
      fkSolverPos(fksolver),
      ikSolverVel(iksolver)
{
    qMin.data.setConstant(std::numeric_limits<double>::min());
    qMax.data.setConstant(std::numeric_limits<double>::max());
}

// -----------------------------------------------------------------------------

int ChainIkSolverPos_ID::CartToJnt(const KDL::JntArray & q_init, const KDL::Frame & p_in, KDL::JntArray & q_out)
{
    if (nj != chain.getNrOfJoints())
    {
        return (error = E_NOT_UP_TO_DATE);
    }

    if (nj != q_init.rows() || nj != q_out.rows() || nj != qMin.rows() || nj != qMax.rows())
    {
        return (error = E_SIZE_MISMATCH);
    }

    KDL::Frame f;

    if (fkSolverPos.JntToCart(q_init, f) < 0)
    {
        return (error = E_FKSOLVERPOS_FAILED);
    }

    KDL::Twist delta_twist = KDL::diff(f, p_in);
    KDL::JntArray delta_q;

    if (ikSolverVel.CartToJnt(q_out, delta_twist, delta_q) < 0)
    {
        return (error = E_IKSOLVERVEL_FAILED);
    }

    KDL::Add(q_init, delta_q, q_out);

    for (unsigned int j = 0; j < qMin.rows(); j++)
    {
        if (q_out(j) < qMin(j))
        {
            q_out(j) = qMin(j);
        }
    }

    for (unsigned int j = 0; j < qMax.rows(); j++)
    {
        if (q_out(j) > qMax(j))
        {
            q_out(j) = qMax(j);
        }
    }

    return (error = E_NOERROR);
}

// -----------------------------------------------------------------------------

void ChainIkSolverPos_ID::updateInternalDataStructures()
{
    nj = chain.getNrOfJoints();
    qMin.data.conservativeResizeLike(Eigen::VectorXd::Constant(nj, std::numeric_limits<double>::min()));
    qMax.data.conservativeResizeLike(Eigen::VectorXd::Constant(nj, std::numeric_limits<double>::max()));
    fkSolverPos.updateInternalDataStructures();
    ikSolverVel.updateInternalDataStructures();
}

// -----------------------------------------------------------------------------

const char * ChainIkSolverPos_ID::strError(const int error) const
{
    switch (error)
    {
    case E_FKSOLVERPOS_FAILED:
        return "Internal FK position solver failed";
    case E_IKSOLVERVEL_FAILED:
        return "Internal IK velocity solver failed";
    default:
        return KDL::SolverI::strError(error);
    }
}

// -----------------------------------------------------------------------------

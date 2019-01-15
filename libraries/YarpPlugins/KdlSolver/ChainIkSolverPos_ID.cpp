// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ChainIkSolverPos_ID.hpp"

#include <limits>
#include <vector>

#include <kdl/frames.hpp>

#include <Eigen/Core>

using namespace roboticslab;

// -----------------------------------------------------------------------------

ChainIkSolverPos_ID::ChainIkSolverPos_ID(const KDL::Chain & _chain, const KDL::JntArray & _q_min,
        const KDL::JntArray & _q_max, KDL::ChainFkSolverPos & fksolver)
    : chain(_chain),
      nj(chain.getNrOfJoints()),
      qMin(_q_min),
      qMax(_q_max),
      fkSolverPos(fksolver),
      jacSolver(chain),
      jacobian(nj)
{}

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

    if (jacSolver.JntToJac(q_init, jacobian) < 0)
    {
        return (error = E_JACSOLVER_FAILED);
    }

    KDL::JntArray delta_q = computeDiffInvKin(delta_twist);

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

KDL::JntArray ChainIkSolverPos_ID::computeDiffInvKin(const KDL::Twist & delta_twist)
{
    // Samuel R. Buss, "Introduction to Inverse Kinematics with Jacobian Transpose,
    // Pseudoinverse and Damped Least Squares methods", Department of Mathematics,
    // University of California, San Diego [unpublished].

    KDL::JntArray q(nj);

    Eigen::Matrix<double, 6, 1> e;

    e[0] = delta_twist.vel.x();
    e[1] = delta_twist.vel.y();
    e[2] = delta_twist.vel.z();
    e[3] = delta_twist.rot.x();
    e[4] = delta_twist.rot.y();
    e[5] = delta_twist.rot.z();

    Eigen::Matrix<double, 6, 1> JJTe = jacobian.data * jacobian.data.transpose() * e;
    double alpha = e.dot(JJTe) / JJTe.dot(JJTe);

    q.data = alpha * jacobian.data.transpose() * e;

    return q;
}

// -----------------------------------------------------------------------------

void ChainIkSolverPos_ID::updateInternalDataStructures()
{
    nj = chain.getNrOfJoints();
    qMin.data.conservativeResizeLike(Eigen::VectorXd::Constant(nj, std::numeric_limits<double>::min()));
    qMax.data.conservativeResizeLike(Eigen::VectorXd::Constant(nj, std::numeric_limits<double>::max()));
    fkSolverPos.updateInternalDataStructures();
    jacSolver.updateInternalDataStructures();
    jacobian.resize(nj);
}

// -----------------------------------------------------------------------------

const char * ChainIkSolverPos_ID::strError(const int error) const
{
    switch (error)
    {
    case E_FKSOLVERPOS_FAILED:
        return "Internal FK position solver failed";
    case E_JACSOLVER_FAILED:
        return "Internal Jacobian solver failed";
    default:
        return KDL::SolverI::strError(error);
    }
}

// -----------------------------------------------------------------------------

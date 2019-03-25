// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHAIN_IK_SOLVER_POS_ID_HPP__
#define __CHAIN_IK_SOLVER_POS_ID_HPP__

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace roboticslab
{

/**
 * @ingroup KdlSolver
 * @brief IK solver using infinitesimal displacement twists.
 *
 * Re-implementation of KDL::ChainIkSolverPos_NR_JL in which only one iteration step
 * is performed. Aimed to provide a quick means of obtaining IK whenever the displacements
 * are small enough.
 */
class ChainIkSolverPos_ID : public KDL::ChainIkSolverPos
{
public:

    /**
     * @brief Constructor
     *
     * @param chain The chain to calculate the inverse position for.
     * @param q_min The minimum joint positions.
     * @param q_max The maximum joint positions.
     * @param fksolver A forward position kinematics solver.
     * @param iksolver An inverse velocity kinematics solver.
     */
    ChainIkSolverPos_ID(const KDL::Chain & chain, const KDL::JntArray & q_min, const KDL::JntArray & q_max, KDL::ChainFkSolverPos & fksolver);

    /**
     * @brief Calculate inverse position kinematics.
     *
     * @param q_init Initial guess of the joint coordinates (unused).
     * @param p_in Input cartesian coordinates.
     * @param q_out Output joint coordinates.
     *
     * @return Return code, \ref E_SOLUTION_NOT_FOUND if there is no solution or
     * \ref E_NOT_REACHABLE if at least one of them is out of reach.
     */
    virtual int CartToJnt(const KDL::JntArray & q_init, const KDL::Frame & p_in, KDL::JntArray & q_out);

    /**
    * @brief Update the internal data structures.
    *
    * Update the internal data structures. This is required if the number of segments
    * or number of joints of a chain has changed. This provides a single point of contact
    * for solver memory allocations.
    */
    virtual void updateInternalDataStructures();

    /**
     * @brief Return a description of the last error
     *
     * @param error Error code.
     *
     * @return If \p error is known then a description of \p error, otherwise
     * "UNKNOWN ERROR".
     */
    virtual const char * strError(const int error) const;

    /** @brief Return code, internal FK position solver failed. */
    static const int E_FKSOLVERPOS_FAILED = -100;

    /** @brief Return code, internal Jacobian solver failed. */
    static const int E_JACSOLVER_FAILED = -101;

private:

    KDL::JntArray computeDiffInvKin(const KDL::Twist & delta_twist);

    const KDL::Chain & chain;
    unsigned int nj;

    KDL::JntArray qMin;
    KDL::JntArray qMax;

    KDL::ChainFkSolverPos & fkSolverPos;
    KDL::ChainJntToJacSolver jacSolver;

    KDL::Jacobian jacobian;
};

}  // namespace roboticslab

#endif  // __CHAIN_IK_SOLVER_POS_ID_HPP__

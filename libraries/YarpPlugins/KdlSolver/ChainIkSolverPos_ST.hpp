// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHAIN_IK_SOLVER_POS_ST_HPP__
#define __CHAIN_IK_SOLVER_POS_ST_HPP__

#include <kdl/chainiksolver.hpp>

#include "ScrewTheoryIkProblem.hpp"
#include "ConfigurationSelector.hpp"

namespace roboticslab
{

/**
 * @ingroup KdlSolver
 * @brief IK solver using Screw Theory.
 *
 * Implementation of an inverse position kinematics algorithm. This is a thin wrapper
 * around \ref ScrewTheoryIkProblem. Non-exhaustive tests on TEO's (UC3M) right arm
 * kinematic chain reveal that this is 5-10 faster than a numeric Newton-Raphson
 * solver as provided by KDL (e.g. KDL::ChainIkSolverPos_NR_JL).
 */
class ChainIkSolverPos_ST : public KDL::ChainIkSolverPos
{
public:

    /** @brief Destructor. */
    virtual ~ChainIkSolverPos_ST();

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

    /**
     * @brief Create an instance of \ref ChainIkSolverPos_ST.
     *
     * @param chain Input kinematic chain.
     * @param configFactory Instance of an abstract factory class that
     * instantiates a ConfigurationSelector.
     *
     * @return Solver instance or NULL if no solution was found.
     */
    static KDL::ChainIkSolverPos * create(const KDL::Chain & chain, const ConfigurationSelectorFactory & configFactory);

    /** @brief Return code, IK solution not found. */
    static const int E_SOLUTION_NOT_FOUND = -100;

    /** @brief Return code, target pose out of robot limits. */
    static const int E_OUT_OF_LIMITS = -101;

    /** @brief Return code, solution out of reach. */
    static const int E_NOT_REACHABLE = 100;

private:

    ChainIkSolverPos_ST(const KDL::Chain & chain, ScrewTheoryIkProblem * problem, ConfigurationSelector * config);

    const KDL::Chain & chain;

    ScrewTheoryIkProblem * problem;

    ConfigurationSelector * config;
};

}  // namespace roboticslab

#endif  // __CHAIN_IK_SOLVER_POS_ST_HPP__

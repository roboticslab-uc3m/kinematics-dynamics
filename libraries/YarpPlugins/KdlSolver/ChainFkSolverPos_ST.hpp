// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHAIN_FK_SOLVER_POS_ST_HPP__
#define __CHAIN_FK_SOLVER_POS_ST_HPP__

#include <kdl/chainfksolver.hpp>

#include "ProductOfExponentials.hpp"

namespace roboticslab
{

/**
 * @ingroup KdlSolver
 * @brief FK solver using Screw Theory.
 *
 * Implementation of a forward position kinematics algorithm. This is a thin wrapper
 * around \ref PoeExpression. Methods that retrieve resulting frames for intermediate
 * links are not supported.
 */
class ChainFkSolverPos_ST : public KDL::ChainFkSolverPos
{
public:

    /**
     * @brief Perform FK on the selected segment
     *
     * @param q_in Input joint coordinates.
     * @param p_out Reference to output cartesian pose.
     * @param segmentNr Desired segment frame (unsupported).
     *
     * @return Return code, < 0 if something went wrong.
     */
    virtual int JntToCart(const KDL::JntArray & q_in, KDL::Frame & p_out, int segmentNr = -1);

    /**
     * @brief Perform FK on the selected segments (unsupported)
     *
     * @param q_in Input joint coordinates.
     * @param p_out Reference to a vector of output cartesian poses for all segments.
     * @param segmentNr Last selected segment frame.
     *
     * @return Return code, < 0 if something went wrong.
     *
     * @warning Unsupported, will return @ref E_OPERATION_NOT_SUPPORTED.
     */
    virtual int JntToCart(const KDL::JntArray & q_in, std::vector<KDL::Frame> & p_out, int segmentNr = -1);

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
     * @brief Create an instance of \ref ChainFkSolverPos_ST.
     *
     * @param chain Input kinematic chain.
     *
     * @return Solver instance.
     */
    static KDL::ChainFkSolverPos * create(const KDL::Chain & chain);

    /** @brief Return code, operation not supported. */
    static const int E_OPERATION_NOT_SUPPORTED = -100;

    /** @brief Return code, input vector size does not match expected output vector size. */
    static const int E_ILLEGAL_ARGUMENT_SIZE = -101;

private:

    ChainFkSolverPos_ST(const KDL::Chain & chain);

    const KDL::Chain & chain;

    PoeExpression poe;
};

}  // namespace roboticslab

#endif  // __CHAIN_FK_SOLVER_POS_ST_HPP__

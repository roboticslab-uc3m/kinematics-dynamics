// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHAIN_IK_SOLVER_POS_ST_HPP__
#define __CHAIN_IK_SOLVER_POS_ST_HPP__

#include <kdl/chainiksolver.hpp>

#include "ScrewTheoryIkProblem.hpp"

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ChainIkSolverPos_ST : public KDL::ChainIkSolverPos
{
public:

    ~ChainIkSolverPos_ST();

    virtual int CartToJnt(const KDL::JntArray & q_init, const KDL::Frame & p_in, KDL::JntArray & q_out);

    virtual void updateInternalDataStructures();

    virtual const char * strError(const int error) const;

    static KDL::ChainIkSolverPos * create(const KDL::Chain & chain);

    static const int E_SOLUTION_NOT_FOUND = -100;

private:

    ChainIkSolverPos_ST(const KDL::Chain & chain, ScrewTheoryIkProblem * problem);

    const KDL::Chain & chain;

    ScrewTheoryIkProblem * problem;
};

}  // namespace roboticslab

#endif  // __CHAIN_IK_SOLVER_POS_ST_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHAIN_FK_SOLVER_POS_ST_HPP__
#define __CHAIN_FK_SOLVER_POS_ST_HPP__

#include <kdl/chainfksolver.hpp>

#include "ProductOfExponentials.hpp"

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ChainFkSolverPos_ST : public KDL::ChainFkSolverPos
{
public:

    virtual int JntToCart(const KDL::JntArray & q_in, KDL::Frame & p_out, int segmentNr = -1);

    virtual int JntToCart(const KDL::JntArray & q_in, std::vector<KDL::Frame> & p_out, int segmentNr = -1);

    virtual void updateInternalDataStructures();

    virtual const char * strError(const int error) const;

    static KDL::ChainFkSolverPos * create(const KDL::Chain & chain);

    static const int E_OPERATION_NOT_SUPPORTED = -100;

    static const int E_ILLEGAL_ARGUMENT_SIZE = -101;

private:

    ChainFkSolverPos_ST(const KDL::Chain & chain);

    const KDL::Chain & chain;

    PoeExpression poe;
};

}  // namespace roboticslab

#endif  // __CHAIN_FK_SOLVER_POS_ST_HPP__

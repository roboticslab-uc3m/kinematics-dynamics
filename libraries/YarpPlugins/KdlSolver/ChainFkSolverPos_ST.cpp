// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ChainFkSolverPos_ST.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

ChainFkSolverPos_ST::ChainFkSolverPos_ST(const KDL::Chain & _chain)
    : chain(_chain),
      poe(PoeExpression::fromChain(chain))
{}

// -----------------------------------------------------------------------------

int ChainFkSolverPos_ST::JntToCart(const KDL::JntArray & q_in, KDL::Frame & p_out, int segmentNr)
{
    if (segmentNr >= 0)
    {
        return (error = E_OPERATION_NOT_SUPPORTED);
    }

    if (!poe.evaluate(q_in, p_out))
    {
        return (error = E_ILLEGAL_ARGUMENT_SIZE);
    }

    return (error = E_NOERROR);
}

// -----------------------------------------------------------------------------

int ChainFkSolverPos_ST::JntToCart(const KDL::JntArray & q_in, std::vector<KDL::Frame> & p_out, int segmentNr)
{
    return (error = E_OPERATION_NOT_SUPPORTED);
}

// -----------------------------------------------------------------------------

void ChainFkSolverPos_ST::updateInternalDataStructures()
{
    poe = PoeExpression::fromChain(chain);
}

// -----------------------------------------------------------------------------

const char * ChainFkSolverPos_ST::strError(const int error) const
{
    switch (error)
    {
    case E_OPERATION_NOT_SUPPORTED:
        return "Unsupported operation";
    case E_ILLEGAL_ARGUMENT_SIZE:
        return "Illegal argument size";
    default:
        return KDL::SolverI::strError(error);
    }
}

// -----------------------------------------------------------------------------

KDL::ChainFkSolverPos * ChainFkSolverPos_ST::create(const KDL::Chain & chain)
{
    return new ChainFkSolverPos_ST(chain);
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ChainIkSolverPos_ST.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

ChainIkSolverPos_ST::ChainIkSolverPos_ST(const KDL::Chain & _chain, ScrewTheoryIkProblem * _problem)
    : chain(_chain),
      problem(_problem)
{}

// -----------------------------------------------------------------------------

ChainIkSolverPos_ST::~ChainIkSolverPos_ST()
{
    delete problem;
    problem = NULL;
}

// -----------------------------------------------------------------------------

int ChainIkSolverPos_ST::CartToJnt(const KDL::JntArray & q_init, const KDL::Frame & p_in, KDL::JntArray & q_out)
{
    std::vector<KDL::JntArray> solutions;

    if (error == E_SOLUTION_NOT_FOUND)
    {
        return error;
    }

    int ret = problem->solve(p_in, solutions);

    q_out = solutions.at(0); // TODO

    if (ret == E_NOT_REACHABLE)
    {
        return (error = ret);
    }
    else
    {
        return (error = E_NOERROR);
    }
}

// -----------------------------------------------------------------------------

void ChainIkSolverPos_ST::updateInternalDataStructures()
{
    PoeExpression poe = PoeExpression::fromChain(chain);
    ScrewTheoryIkProblemBuilder builder(poe);
    ScrewTheoryIkProblem * problem = builder.build();

    if (problem == NULL)
    {
        error = E_SOLUTION_NOT_FOUND;
        return;
    }

    delete this->problem;
    this->problem = problem;
}

// -----------------------------------------------------------------------------

KDL::ChainIkSolverPos * ChainIkSolverPos_ST::create(const KDL::Chain & chain)
{
    PoeExpression poe = PoeExpression::fromChain(chain);
    ScrewTheoryIkProblemBuilder builder(poe);
    ScrewTheoryIkProblem * problem = builder.build();

    if (problem == NULL)
    {
        return NULL;
    }

    return new ChainIkSolverPos_ST(chain, problem);
}

// -----------------------------------------------------------------------------

const char * ChainIkSolverPos_ST::strError(const int error) const
{
    switch (error)
    {
    case E_SOLUTION_NOT_FOUND:
        return "IK solution not found";
    case E_NOT_REACHABLE:
        return "IK solution not reachable";
    default:
        return KDL::SolverI::strError(error);
    }
}

// -----------------------------------------------------------------------------

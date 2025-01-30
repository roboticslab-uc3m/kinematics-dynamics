// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ChainIkSolverPos_ST.hpp"

#include <algorithm> // std::transform
#include <iterator> // std::back_inserter

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

ChainIkSolverPos_ST::ChainIkSolverPos_ST(const KDL::Chain & _chain, ScrewTheoryIkProblem * _problem,
        ConfigurationSelector * _config)
    : chain(_chain),
      problem(_problem),
      config(_config)
{}

// -----------------------------------------------------------------------------

ChainIkSolverPos_ST::~ChainIkSolverPos_ST()
{
    delete problem;
    problem = nullptr;

    delete config;
    config = nullptr;
}

// -----------------------------------------------------------------------------

int ChainIkSolverPos_ST::CartToJnt(const KDL::JntArray & q_init, const KDL::Frame & p_in, KDL::JntArray & q_out)
{
    if (error == E_SOLUTION_NOT_FOUND)
    {
        return error;
    }

    std::vector<KDL::JntArray> solutions;

    bool ret = problem->solve(p_in, q_init, solutions);

    if (!config->configure(solutions))
    {
        return (error = E_OUT_OF_LIMITS);
    }

    if (!config->findOptimalConfiguration(q_init))
    {
        return (error = E_OUT_OF_LIMITS);
    }

    config->retrievePose(q_out);

    return (error = ret ? E_NOERROR : E_NOT_REACHABLE);
}

// -----------------------------------------------------------------------------

void ChainIkSolverPos_ST::updateInternalDataStructures()
{
    PoeExpression poe = PoeExpression::fromChain(chain);
    ScrewTheoryIkProblemBuilder builder(poe);
    ScrewTheoryIkProblem * problem = builder.build();

    if (!problem)
    {
        error = E_SOLUTION_NOT_FOUND;
        return;
    }

    delete this->problem;
    this->problem = problem;
}

// -----------------------------------------------------------------------------

KDL::ChainIkSolverPos * ChainIkSolverPos_ST::create(const KDL::Chain & chain, const ConfigurationSelectorFactory & configFactory)
{
    PoeExpression poe = PoeExpression::fromChain(chain);
    ScrewTheoryIkProblemBuilder builder(poe);
    ScrewTheoryIkProblem * problem = builder.build();

    if (!problem)
    {
        return nullptr;
    }

    const auto & steps = problem->getSteps();

    std::vector<const char *> descriptions;
    std::transform(steps.cbegin(), steps.cend(), std::back_inserter(descriptions), [](const auto & step) { return step.second->describe(); });
    yCInfo(KDLS) << "Found" << problem->solutions() << "solutions:" << descriptions << (problem->isReversed() ? "(reversed)" : "");

    ConfigurationSelector * config = configFactory.create();

    return new ChainIkSolverPos_ST(chain, problem, config);
}

// -----------------------------------------------------------------------------

const char * ChainIkSolverPos_ST::strError(const int error) const
{
    switch (error)
    {
    case E_SOLUTION_NOT_FOUND:
        return "IK solution not found";
    case E_OUT_OF_LIMITS:
        return "Target pose out of robot limits";
    case E_NOT_REACHABLE:
        return "IK solution not reachable";
    default:
        return KDL::SolverI::strError(error);
    }
}

// -----------------------------------------------------------------------------

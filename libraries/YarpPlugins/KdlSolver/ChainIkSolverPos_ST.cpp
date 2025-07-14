// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ChainIkSolverPos_ST.hpp"

#include <algorithm> // std::none_of, std::transform
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

    auto reachability = problem->solve(p_in, q_init, solutions);

    // if (std::none_of(reachability.begin(), reachability.end(), [](bool r) { return r; }))
    // {
    //     return (error = E_NOT_REACHABLE);
    // }

    // if (!config->configure(solutions, reachability))
    // {
    //     return (error = E_OUT_OF_LIMITS);
    // }

    // if (!config->findOptimalConfiguration(q_init))
    // {
    //     return (error = E_OUT_OF_LIMITS);
    // }

    // config->retrievePose(q_out);

    for (const auto & s : solutions)
    {
        std::printf("%f %f %f %f %f %f\n", s(0), s(1), s(2), s(3), s(4), s(5));
    }

    q_out = solutions[0];

    return (error = E_NOERROR);
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

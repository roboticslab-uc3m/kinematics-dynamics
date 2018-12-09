// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkProblem.hpp"

#include <algorithm>
#include <functional>
#include <numeric>

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline double getTheta(const KDL::JntArray & q, int i, bool reversed)
    {
        return reversed ? -q(q.rows() - 1 - i) : q(i);
    }

    struct solution_accumulator : std::binary_function<int, const ScrewTheoryIkSubproblem *, int>
    {
        result_type operator()(first_argument_type count, const second_argument_type & subproblem)
        {
            return count * subproblem->solutions();
        }
    }
    solutionAccumulator;

    inline int computeSolutions(const ScrewTheoryIkProblem::Steps & steps)
    {
        if (!steps.empty())
        {
            return std::accumulate(steps.begin(), steps.end(), 1, solutionAccumulator);
        }
        else
        {
            return 0;
        }
    }
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem::ScrewTheoryIkProblem(const PoeExpression & _poe, const Steps & _steps, bool _reversed)
    : poe(_poe),
      steps(_steps),
      reversed(_reversed),
      soln(computeSolutions(steps))
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem::~ScrewTheoryIkProblem()
{
    for (int i = 0; i < steps.size(); i++)
    {
        delete steps[i];
    }
}

// -----------------------------------------------------------------------------

bool ScrewTheoryIkProblem::solve(const KDL::Frame & H_S_T, Solutions & solutions)
{
    if (!solutions.empty())
    {
        solutions.clear();
    }

    solutions.reserve(soln);
    solutions.push_back(KDL::JntArray(poe.size()));

    PoeTerms poeTerms(poe.size(), EXP_UNKNOWN);

    Frames rhsFrames;

    rhsFrames.reserve(soln);
    rhsFrames.push_back((reversed ? H_S_T.Inverse() : H_S_T) * poe.getTransform().Inverse());

    bool firstIteration = true;

    for (int i = 0; i < steps.size(); i++)
    {
        if (!firstIteration)
        {
            recalculateFrames(solutions, rhsFrames, poeTerms);
        }

        int previousSize = solutions.size();

        for (int j = 0; j < previousSize; j++)
        {
            const KDL::Frame & H = transformPoint(solutions[j], poeTerms);
            const ScrewTheoryIkSubproblem::Solutions & partialSolutions = steps[i]->solve(rhsFrames[j], H);

            if (partialSolutions.size() > 1)
            {
                solutions.resize(previousSize * partialSolutions.size());
                rhsFrames.resize(previousSize * partialSolutions.size());

                for (int k = 1; k < partialSolutions.size(); k++)
                {
                    solutions[j + previousSize * k] = solutions[j];
                    rhsFrames[j + previousSize * k] = rhsFrames[j];
                }
            }

            for (int k = 0; k < partialSolutions.size(); k++)
            {
                const ScrewTheoryIkSubproblem::JointIdsToSolutions & jointIdsToSolutions = partialSolutions[k];

                for (int l = 0; l < jointIdsToSolutions.size(); l++)
                {
                    const ScrewTheoryIkSubproblem::JointIdToSolution & jointIdToSolution = jointIdsToSolutions[l];

                    int id = jointIdToSolution.first;
                    double theta = jointIdToSolution.second;

                    poeTerms[id] = EXP_KNOWN;

                    if (reversed)
                    {
                        id = poe.size() - 1 - id;
                        theta = -theta;
                    }

                    solutions[j + previousSize * k](id) = theta;
                }
            }
        }

        firstIteration = false;
    }

    return std::count(poeTerms.begin(), poeTerms.end(), EXP_UNKNOWN) == 0;
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblem::recalculateFrames(const Solutions & solutions, Frames & frames, PoeTerms & poeTerms)
{
    std::vector<KDL::Frame> pre, post;

    if (recalculateFrames(solutions, pre, poeTerms, false))
    {
        for (int i = 0; i < solutions.size(); i++)
        {
            frames[i] = pre[i].Inverse() * frames[i];
        }
    }

    if (recalculateFrames(solutions, post, poeTerms, true))
    {
        for (int i = 0; i < solutions.size(); i++)
        {
            frames[i] = frames[i] * post[i].Inverse();
        }
    }
}

// -----------------------------------------------------------------------------

bool ScrewTheoryIkProblem::recalculateFrames(const Solutions & solutions, Frames & frames, PoeTerms & poeTerms, bool backwards)
{
    frames.resize(solutions.size());

    bool hasMultipliedTerms = false;

    for (int i = 0; i < poeTerms.size(); i++)
    {
        if (backwards)
        {
            i = poeTerms.size() - 1 - i;
        }

        if (poeTerms[i] == EXP_KNOWN)
        {
            for (int j = 0; j < solutions.size(); j++)
            {
                const MatrixExponential & exp = poe.exponentialAtJoint(i);
                frames[j] = frames[j] * exp.asFrame(getTheta(solutions[j], i, reversed));
            }

            poeTerms[i] = EXP_COMPUTED;
            hasMultipliedTerms = true;
        }
        else if (poeTerms[i] == EXP_COMPUTED)
        {
            continue;
        }
        else
        {
            break;
        }
    }

    return hasMultipliedTerms;
}

// -----------------------------------------------------------------------------

KDL::Frame ScrewTheoryIkProblem::transformPoint(const KDL::JntArray & jointValues, const PoeTerms & poeTerms)
{
    KDL::Frame H;

    bool foundKnown = false;
    bool foundUnknown = false;

    for (int i = poeTerms.size() - 1; i >= 0; i--)
    {
        if (poeTerms[i] == EXP_KNOWN)
        {
            const MatrixExponential & exp = poe.exponentialAtJoint(i);
            H = exp.asFrame(getTheta(jointValues, i, reversed)) * H;
            foundKnown = true;
        }
        else if (poeTerms[i] == EXP_UNKNOWN)
        {
            foundUnknown = true;

            if (foundKnown)
            {
                break;
            }
        }
        else if (poeTerms[i] == EXP_COMPUTED)
        {
            if (foundKnown || foundUnknown)
            {
                break;
            }
        }
    }

    return H;
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem * ScrewTheoryIkProblem::create(const PoeExpression & poe, const Steps & steps, bool reversed)
{
    // TODO: validate
    return new ScrewTheoryIkProblem(poe, steps, reversed);
}

// -----------------------------------------------------------------------------

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

    // Reserve space in memory to avoid additional allocations on runtime.
    solutions.reserve(soln);
    solutions.push_back(KDL::JntArray(poe.size()));

    PoeTerms poeTerms(poe.size(), EXP_UNKNOWN);

    Frames rhsFrames;

    rhsFrames.reserve(soln);
    rhsFrames.push_back((reversed ? H_S_T.Inverse() : H_S_T) * poe.getTransform().Inverse());

    bool firstIteration = true;
    bool reachable = true;

    for (int i = 0; i < steps.size(); i++)
    {
        if (!firstIteration)
        {
            // Re-compute right-hand side of PoE equation, i.e. prod(e_i) = H_S_T_q * H_S_T_0^(-1)
            recalculateFrames(solutions, rhsFrames, poeTerms);
        }

        // Save this, the vector of solutions might be resized in the following loop, increase size
        // on demand for improved efficiency (instead of allocating everything from the start).
        int previousSize = solutions.size();

        for (int j = 0; j < previousSize; j++)
        {
            // Apply known frames to the first characteristic point for each subproblem.
            const KDL::Frame & H = transformPoint(solutions[j], poeTerms);

            ScrewTheoryIkSubproblem::Solutions partialSolutions;

            // Actually solve each subproblem, use current right-hand side of PoE to obtain
            // the right-hand side of said subproblem.
            reachable = reachable & steps[i]->solve(rhsFrames[j], H, partialSolutions);

            // The global number of solutions is increased by this step.
            if (partialSolutions.size() > 1)
            {
                // Noop if current size is not less than requested.
                solutions.resize(previousSize * partialSolutions.size());
                rhsFrames.resize(previousSize * partialSolutions.size());

                for (int k = 1; k < partialSolutions.size(); k++)
                {
                    // Replicate known solutions, these won't change further on.
                    solutions[j + previousSize * k] = solutions[j];

                    // Replicate right-hand side frames for the next iteration, these might change.
                    rhsFrames[j + previousSize * k] = rhsFrames[j];
                }
            }

            // For each local solution of this subproblem...
            for (int k = 0; k < partialSolutions.size(); k++)
            {
                const ScrewTheoryIkSubproblem::JointIdsToSolutions & jointIdsToSolutions = partialSolutions[k];

                // For each joint-id-to-value pair of this local solution...
                for (int l = 0; l < jointIdsToSolutions.size(); l++)
                {
                    const ScrewTheoryIkSubproblem::JointIdToSolution & jointIdToSolution = jointIdsToSolutions[l];

                    int id = jointIdToSolution.first;
                    double theta = jointIdToSolution.second;

                    // Preserve mapping of ids (associated to `poe`).
                    poeTerms[id] = EXP_KNOWN;

                    if (reversed)
                    {
                        id = poe.size() - 1 - id;
                        theta = -theta;
                    }

                    // Store the final value in the desired index, don't shuffle it after this point.
                    solutions[j + previousSize * k](id) = theta;
                }
            }
        }

        firstIteration = false;
    }

    return reachable;
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblem::recalculateFrames(const Solutions & solutions, Frames & frames, PoeTerms & poeTerms)
{
    std::vector<KDL::Frame> pre, post;

    // Leftmost known terms of the PoE.
    if (recalculateFrames(solutions, pre, poeTerms, false))
    {
        for (int i = 0; i < solutions.size(); i++)
        {
            frames[i] = pre[i].Inverse() * frames[i];
        }
    }

    // Rightmost known terms of the PoE.
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

            // Mark as 'computed' and include in right-hand side of PoE so that this
            // term will be ignored in future iterations.
            poeTerms[i] = EXP_COMPUTED;
            hasMultipliedTerms = true;
        }
        else if (poeTerms[i] == EXP_COMPUTED)
        {
            continue;
        }
        else
        {
            // We hit an unknown term, quit this loop.
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
                // Already applied at least one transformation, can't proceed further.
                break;
            }
        }
        else if (poeTerms[i] == EXP_COMPUTED)
        {
            if (foundKnown || foundUnknown)
            {
                // Only skip this if we have a sequence of aldeady-computed terms in the
                // rightmost end of the PoE.
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

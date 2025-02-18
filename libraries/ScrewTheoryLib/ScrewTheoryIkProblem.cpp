// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkProblem.hpp"

#include <numeric> // std::accumulate
#include <vector>

#include "ScrewTheoryTools.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline double getTheta(const KDL::JntArray & q, int i, bool reversed)
    {
        return reversed ? -q(q.rows() - 1 - i) : q(i);
    }

    struct solution_accumulator
    {
        inline int operator()(int count, const ScrewTheoryIkProblem::JointIdsToSubproblem & idToSubproblem)
        {
            return count * idToSubproblem.second->solutions();
        }
    }
    solutionAccumulator;

    inline int computeSolutions(const ScrewTheoryIkProblem::Steps & steps)
    {
        return !steps.empty() ? std::accumulate(steps.begin(), steps.end(), 1, solutionAccumulator) : 0;
    }

    std::vector<double> extractValues(const std::vector<int> indices, const KDL::JntArray & q, bool reversed)
    {
        std::vector<double> values(indices.size());

        for (auto i = 0; i < indices.size(); i++)
        {
            if (!reversed)
            {
                values[i] = q(indices[i]);
            }
            else
            {
                values[i] = -q(q.rows() - 1 - indices[i]);
            }
        }

        return values;
    }
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem::ScrewTheoryIkProblem(const PoeExpression & _poe, const Steps & _steps, bool _reversed)
    : poe(_poe),
      steps(_steps),
      reversed(_reversed),
      soln(computeSolutions(steps))
{
    poeTerms.reserve(soln);
    rhsFrames.reserve(soln);
    reachability.reserve(soln);
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem::~ScrewTheoryIkProblem()
{
    for (auto & step : steps)
    {
        delete step.second;
    }
}

// -----------------------------------------------------------------------------

std::vector<bool> ScrewTheoryIkProblem::solve(const KDL::Frame & H_S_T, const KDL::JntArray & reference, Solutions & solutions)
{
    solutions.clear();
    rhsFrames.clear();
    reachability.clear();

    // Reserve space in memory to avoid additional allocations on runtime.
    solutions.reserve(soln);

    // Insert a dummy value to avoid accessing an empty vector.
    solutions.emplace_back(poe.size());
    rhsFrames.emplace_back((reversed ? H_S_T.Inverse() : H_S_T) * poe.getTransform().Inverse());
    reachability.emplace_back(true);

    poeTerms.assign(poe.size(), EXP_UNKNOWN);

    bool firstIteration = true;

    ScrewTheoryIkSubproblem::Solutions partialSolutions;

    for (const auto [ids, subproblem] : steps)
    {
        const auto referenceValues = extractValues(ids, reference, reversed);

        if (!firstIteration)
        {
            // Re-compute right-hand side of PoE equation, i.e. prod(e_i) = H_S_T_q * H_S_T_0^(-1)
            recalculateFrames(solutions, rhsFrames, poeTerms);
        }

        // Save this, the vector of solutions might be resized in the following loop, increase size
        // on demand for improved efficiency (instead of allocating everything from the start).
        int previousSize = solutions.size();

        for (int i = 0; i < previousSize; i++)
        {
            // Apply known frames to the first characteristic point for each subproblem.
            const KDL::Frame & H = transformPoint(solutions[i], poeTerms);

            // Actually solve each subproblem, use current right-hand side of PoE to obtain
            // the right-hand side of said subproblem. Local reachability is common to all
            // partial solutions, and will be and-ed with the global reachability status.
            bool reachable = subproblem->solve(rhsFrames[i], H, referenceValues, partialSolutions) & reachability[i];

            // The global number of solutions is increased by this step.
            if (partialSolutions.size() > 1)
            {
                const int partialSize = previousSize * partialSolutions.size();

                // Noop if current size is not less than requested.
                solutions.resize(partialSize);
                rhsFrames.resize(partialSize);
                reachability.resize(partialSize);

                for (int j = 1; j < partialSolutions.size(); j++)
                {
                    const int index = i + previousSize * j;

                    // Replicate known solutions, these won't change further on.
                    solutions[index] = solutions[i];

                    // Replicate right-hand side frames for the next iteration, these might change.
                    rhsFrames[index] = rhsFrames[i];

                    // Replicate reachability status.
                    reachability[index] = reachability[i];
                }
            }

            // For each local solution of this subproblem...
            for (int j = 0; j < partialSolutions.size(); j++)
            {
                // For each joint-id-to-value pair of this local solution...
                for (int k = 0; k < ids.size(); k++)
                {
                    auto id = ids[k];
                    auto theta = partialSolutions[j][k];

                    // Preserve mapping of ids (associated to `poe`).
                    poeTerms[id] = EXP_KNOWN;

                    if (reversed)
                    {
                        id = poe.size() - 1 - id;
                        theta = normalizeAngle(-theta);
                    }

                    // Store the final value in the desired index, don't shuffle it after this point.
                    solutions[i + previousSize * j](id) = theta;
                }

                // Store reachability status.
                reachability[i + previousSize * j] = reachable;
            }
        }

        firstIteration = false;
    }

    return reachability; // returns a copy
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblem::recalculateFrames(const Solutions & solutions, Frames & frames, PoeTerms & poeTerms)
{
    // Leftmost known terms of the PoE.
    if (std::vector<KDL::Frame> pre; recalculateFrames(solutions, pre, poeTerms, false))
    {
        for (int i = 0; i < solutions.size(); i++)
        {
            frames[i] = pre[i].Inverse() * frames[i];
        }
    }

    // Rightmost known terms of the PoE.
    if (std::vector<KDL::Frame> post; recalculateFrames(solutions, post, poeTerms, true))
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
                const auto & exp = poe.exponentialAtJoint(i);
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
            const auto & exp = poe.exponentialAtJoint(i);
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
                // Only skip this if we have a sequence of already-computed terms in the
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

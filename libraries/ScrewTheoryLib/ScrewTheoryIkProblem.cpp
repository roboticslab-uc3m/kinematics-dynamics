// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkProblem.hpp"

#include <algorithm>

using namespace roboticslab;

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem::ScrewTheoryIkProblem(const PoeExpression & _poe, const std::vector<ScrewTheoryIkSubproblem *> & _steps, bool _reversed)
    : poe(_poe),
      steps(_steps),
      reversed(_reversed)
{}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem::~ScrewTheoryIkProblem()
{
    for (int i = 0; i < steps.size(); i++)
    {
        delete steps[i];
        steps[i] = NULL;
    }
}

// -----------------------------------------------------------------------------

bool ScrewTheoryIkProblem::solve(const KDL::Frame & H_S_T, std::vector<KDL::JntArray> & solutions)
{
    solutions.clear();
    solutions.push_back(KDL::JntArray(poe.size()));

    poeTerms.assign(poe.size(), EXP_UNKNOWN);

    rhsFrames.push_back((reversed ? H_S_T.Inverse() : H_S_T) * poe.getTransform().Inverse());

    bool firstIteration = true;

    for (int i = 0; i < steps.size(); i++)
    {
        if (!firstIteration)
        {
            recalculateFrames(solutions);
        }

        int previousSize = solutions.size();

        for (int j = 0; j < previousSize; j++)
        {
            const KDL::Frame & H = transformPoint(solutions[j]);
            const ScrewTheoryIkSubproblem::SolutionsVector & partialSolutions = steps[i]->solve(rhsFrames[j], H);

            if (partialSolutions.size() > 1)
            {
                solutions.resize(previousSize * partialSolutions.size());
                rhsFrames.resize(solutions.size());

                for (int k = 1; k < partialSolutions.size(); k++)
                {
                    solutions[j + previousSize * k] = solutions[j];
                    rhsFrames[j + previousSize * k] = rhsFrames[j];
                }
            }

            for (int k = 0; k < partialSolutions.size(); k++)
            {
                const ScrewTheoryIkSubproblem::JointIdsToSolutionsVector & jointIdsToSolutions = partialSolutions[k];

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

void ScrewTheoryIkProblem::recalculateFrames(const std::vector<KDL::JntArray> & solutions)
{
    std::vector<KDL::Frame> pre(solutions.size(), KDL::Frame::Identity());
    bool hasPremultipliedTerms = false;

    for (int i = 0; i < poeTerms.size(); i++)
    {
        if (poeTerms[i] == EXP_KNOWN)
        {
            for (int j = 0; j < solutions.size(); j++)
            {
                const MatrixExponential & exp = poe.exponentialAtJoint(i);
                double theta;

                if (reversed)
                {
                    theta = -solutions[j](poe.size() - 1 - i);
                }
                else
                {
                    theta = solutions[j](i);
                }

                pre[j] = pre[j] * exp.asFrame(theta);
            }

            poeTerms[i] = EXP_COMPUTED;
            hasPremultipliedTerms = true;
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

    if (hasPremultipliedTerms)
    {
        for (int i = 0; i < rhsFrames.size(); i++)
        {
            rhsFrames[i] = pre[i].Inverse() * rhsFrames[i];
        }
    }

    std::vector<KDL::Frame> post(solutions.size(), KDL::Frame::Identity());
    bool hasPostmultipliedTerms = false;

    for (int i = poeTerms.size() - 1; i >= 0; i++)
    {
        if (poeTerms[i] == EXP_KNOWN)
        {
            for (int j = 0; j < solutions.size(); j++)
            {
                const MatrixExponential & exp = poe.exponentialAtJoint(i);
                double theta;

                if (reversed)
                {
                    theta = -solutions[j](poe.size() - 1 - i);
                }
                else
                {
                    theta = solutions[j](i);
                }

                post[j] = post[j] * exp.asFrame(theta);
            }

            poeTerms[i] = EXP_COMPUTED;
            hasPostmultipliedTerms = true;
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

    if (hasPostmultipliedTerms)
    {
        for (int i = 0; i < rhsFrames.size(); i++)
        {
            rhsFrames[i] = rhsFrames[i] * post[i].Inverse();
        }
    }
}

// -----------------------------------------------------------------------------

KDL::Frame ScrewTheoryIkProblem::transformPoint(const KDL::JntArray & jointValues)
{
    KDL::Frame H = KDL::Frame::Identity();

    bool foundKnown = false;
    bool foundUnknown = false;

    for (int i = poeTerms.size() - 1; i >= 0; i--)
    {
        if (poeTerms[i] == EXP_KNOWN)
        {
            const MatrixExponential & exp = poe.exponentialAtJoint(i);
            double theta;

            if (reversed)
            {
                theta = -jointValues(poe.size() - 1 - i);
            }
            else
            {
                theta = jointValues(i);
            }

            H = exp.asFrame(theta) * H;
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

ScrewTheoryIkProblem * ScrewTheoryIkProblem::create(const PoeExpression & poe, const std::vector<ScrewTheoryIkSubproblem *> & steps, bool reversed)
{
    // TODO: validate
    return new ScrewTheoryIkProblem(poe, steps, reversed);
}

// -----------------------------------------------------------------------------

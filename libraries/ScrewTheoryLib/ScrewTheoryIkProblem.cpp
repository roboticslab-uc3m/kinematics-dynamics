// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkProblem.hpp"

#include <algorithm>
#include <functional>
#include <iterator>
#include <set>

#include "ScrewTheoryIkSubproblems.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline bool liesOnAxis(const MatrixExponential & exp, const KDL::Vector & p)
    {
        return KDL::Equal((p - exp.getOrigin()) * exp.getAxis(), KDL::Vector::Zero());
    }

    inline bool parallelAxes(const MatrixExponential & exp1, const MatrixExponential & exp2)
    {
        return KDL::Equal(exp1.getAxis() * exp2.getAxis(), KDL::Vector::Zero());
    }

    inline bool colinearAxes(const MatrixExponential & exp1, const MatrixExponential & exp2)
    {
        return parallelAxes(exp1, exp2) && liesOnAxis(exp1, exp2.getOrigin());
    }

    bool intersectingAxes(const MatrixExponential & exp1, const MatrixExponential & exp2, KDL::Vector & p)
    {
        // "Intersection of Two Lines in Three-Space" by Ronald Goldman, University of Waterloo (Waterloo, Ontario, Canada)
        // published in: "Graphic Gems", edited by Andrew S. Glassner, 1 ed., ch. 5, "3D Geometry" (p. 304)
        // referenced in: https://stackoverflow.com/a/565282

        KDL::Vector cross = exp1.getAxis() * exp2.getAxis();
        KDL::Vector diff = exp2.getOrigin() - exp1.getOrigin();

        double den = KDL::pow(cross.Norm(), 2);
        double t = KDL::dot(cross, diff * exp2.getAxis()) / den;
        double s = KDL::dot(cross, diff * exp1.getAxis()) / den;

        KDL::Vector L1 = exp1.getOrigin() + exp1.getAxis() * t;
        KDL::Vector L2 = exp2.getOrigin() + exp2.getAxis() * s;

        p = L1;

        return KDL::Equal(L1, L2);
    }

    struct compare_vectors : public std::binary_function<KDL::Vector, KDL::Vector, bool>
    {
        bool operator()(const KDL::Vector & lhs, const KDL::Vector & rhs)
        {
            if (KDL::Equal(lhs.x(), rhs.x()) || lhs.x() < rhs.x())
            {
                return true;
            }

            if (KDL::Equal(lhs.y(), rhs.y()) || lhs.y() < rhs.y())
            {
                return true;
            }

            if (KDL::Equal(lhs.z(), rhs.z()) || lhs.z() < rhs.z())
            {
                return true;
            }

            return false;
        }
    };
}

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

ScrewTheoryIkProblemBuilder::ScrewTheoryIkProblemBuilder(const PoeExpression & _poe)
    : poe(_poe),
      poeTerms(poe.size(), EXP_UNKNOWN)
{}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblemBuilder::searchPoints()
{
    std::set<KDL::Vector, compare_vectors> set;

    set.insert(KDL::Vector::Zero());

    for (int i = 0; i < poe.size(); i++)
    {
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() != MatrixExponential::ROTATION)
        {
            continue;
        }

        set.insert(exp.getOrigin());

        for (int j = 1; j < poe.size(); j++)
        {
            const MatrixExponential & exp2 = poe.exponentialAtJoint(j);

            if (exp2.getMotionType() != MatrixExponential::ROTATION)
            {
                continue;
            }

            if (parallelAxes(exp, exp2))
            {
                continue;
            }

            if (colinearAxes(exp, exp2))
            {
                continue;
            }

            KDL::Vector p;

            if (intersectingAxes(exp, exp2, p))
            {
                set.insert(p);
            }
        }
    }

    set.insert(poe.getTransform().p);

    points.resize(set.size());
    std::copy(set.begin(), set.end(), points.begin());
}

// -----------------------------------------------------------------------------

bool ScrewTheoryIkProblem::solve(const KDL::Frame & H_S_T, std::vector<KDL::JntArray> & solutions)
{
    solutions.clear();
    solutions.push_back(KDL::JntArray(poe.size()));

    poeTerms.assign(poe.size(), EXP_UNKNOWN);

    rhsFrames.push_back(H_S_T * poe.getTransform().Inverse());

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
            const ScrewTheoryIkSubproblem::SolutionsVector & partialSolutions = steps[i]->solve(rhsFrames[j]);

            if (partialSolutions.size() > 1)
            {
                solutions.resize(previousSize * partialSolutions.size());
                rhsFrames.resize(solutions.size());

                for (int k = 1; k < partialSolutions.size(); k++)
                {
                    solutions[j + previousSize * k] = KDL::JntArray(solutions[j]);
                    rhsFrames[j + previousSize * k] = rhsFrames[j];
                }
            }

            for (int k = 0; k < partialSolutions.size(); k++)
            {
                const ScrewTheoryIkSubproblem::JointIdsToSolutionsVector & jointIdsToSolutions = partialSolutions[k];

                for (int l = 0; l < jointIdsToSolutions.size(); l++)
                {
                    const ScrewTheoryIkSubproblem::JointIdToSolution & jointIdToSolution = jointIdsToSolutions[l];
                    poeTerms[jointIdToSolution.first] = EXP_KNOWN;
                    solutions[j + previousSize * k](jointIdToSolution.first) = jointIdToSolution.second;
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
                pre[j] = pre[j] * exp.asFrame(solutions[j](i));
            }

            poeTerms[i] = EXP_COMPUTED;
            hasPremultipliedTerms = true;
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
                post[j] = post[j] * exp.asFrame(solutions[j](i));
            }

            poeTerms[i] = EXP_COMPUTED;
            hasPostmultipliedTerms = true;
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
            rhsFrames[i] = rhsFrames[i] * post[i];
        }
    }
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem * ScrewTheoryIkProblem::create(const PoeExpression & poe, const std::vector<ScrewTheoryIkSubproblem *> & steps)
{
    ScrewTheoryIkProblem * stProblem;
    // TODO: instantiate, validate
    return stProblem;
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem * ScrewTheoryIkProblemBuilder::build()
{
    // TODO: deallocate memory, implement destructor

    searchPoints();

    testPoints.resize(MAX_SIMPLIFICATION_DEPTH);
    poeTerms.assign(poe.size(), EXP_UNKNOWN);

    std::vector<ScrewTheoryIkSubproblem *> steps;

    std::vector<KDL::Vector>::const_iterator pointIt = points.begin();

    int depth = 0;

    do
    {
        testPoints[depth] = *pointIt;

        resetSimplificationState();

        simplify(depth);

        ScrewTheoryIkSubproblem * subproblem = trySolve();

        if (subproblem != NULL)
        {
            steps.push_back(subproblem);
            pointIt = points.begin();
            depth = 0;
            continue;
        }

        ++pointIt;

        if (pointIt == points.end())
        {
            pointIt = points.begin();
            depth++;
        }
    }
    while (depth < MAX_SIMPLIFICATION_DEPTH && std::count(poeTerms.begin(), poeTerms.end(), EXP_KNOWN) != poe.size());

    if (std::count(poeTerms.begin(), poeTerms.end(), EXP_KNOWN) == poe.size())
    {
        return ScrewTheoryIkProblem::create(poe, steps);
    }
    else
    {
        for (std::vector<ScrewTheoryIkSubproblem *>::iterator it = steps.begin(); it != steps.end(); ++it)
        {
            delete (*it);
            *it = NULL;
        }

        return NULL;
    }
}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem * ScrewTheoryIkProblemBuilder::trySolve()
{
    int unknownsCount = std::count(poeTerms.begin(), poeTerms.end(), EXP_UNKNOWN);

    if (unknownsCount > MAX_SIMPLIFICATION_DEPTH)
    {
        return NULL;
    }

    std::vector<poe_term>::reverse_iterator lastUnknown = std::find(poeTerms.rbegin(), poeTerms.rend(), EXP_UNKNOWN);
    int lastExpId = std::distance(poeTerms.begin(), lastUnknown.base()) - 1;
    const MatrixExponential & lastExp = poe.exponentialAtJoint(lastExpId);

    if (unknownsCount == 1)
    {
        if (testPoints.size() == 1)
        {
            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && !liesOnAxis(lastExp, testPoints[0]))
            {
                poeTerms[lastExpId] = EXP_KNOWN;
                return new PadenKahanOne(lastExpId, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION)
            {
                poeTerms[lastExpId] = EXP_KNOWN;
                return new PardosOne(lastExpId, lastExp, testPoints[0]);
            }
        }

        if (testPoints.size() == 2)
        {
            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && !liesOnAxis(lastExp, testPoints[0])
                    && !liesOnAxis(lastExp, testPoints[1]))
            {
                poeTerms[lastExpId] = EXP_KNOWN;
                return new PadenKahanThree(lastExpId, lastExp, testPoints[0], testPoints[1]);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION)
            {
                poeTerms[lastExpId] = EXP_KNOWN;
                return new PardosThree(lastExpId, lastExp, testPoints[0], testPoints[1]);
            }
        }
    }
    else if (unknownsCount == 2)
    {
        std::vector<poe_term>::reverse_iterator nextToLastUnknown = std::find(lastUnknown, poeTerms.rend(), EXP_UNKNOWN);
        int nextToLastExpId = std::distance(poeTerms.begin(), nextToLastUnknown.base()) - 1;
        const MatrixExponential & nextToLastExp = poe.exponentialAtJoint(nextToLastExpId);

        if (testPoints.size() == 1)
        {
            KDL::Vector _;

            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && nextToLastExp.getMotionType() == MatrixExponential::ROTATION
                    && !parallelAxes(lastExp, nextToLastExp)
                    && intersectingAxes(lastExp, nextToLastExp, _))
            {
                poeTerms[lastExpId] = poeTerms[nextToLastExpId] = EXP_KNOWN;
                return new PadenKahanTwo(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION
                    && nextToLastExp.getMotionType() == MatrixExponential::TRANSLATION
                    && !parallelAxes(lastExp, nextToLastExp)
                    && intersectingAxes(lastExp, nextToLastExp, _))
            {
                poeTerms[lastExpId] = poeTerms[nextToLastExpId] = EXP_KNOWN;
                return new PardosTwo(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && nextToLastExp.getMotionType() == MatrixExponential::ROTATION
                    && parallelAxes(lastExp, nextToLastExp)
                    && !colinearAxes(lastExp, nextToLastExp))
            {
                poeTerms[lastExpId] = poeTerms[nextToLastExpId] = EXP_KNOWN;
                return new PardosFour(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }
        }
    }

    return NULL;
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblemBuilder::resetSimplificationState()
{
    for (std::vector<poe_term>::iterator it = poeTerms.begin(); it != poeTerms.end(); ++it)
    {
        if (*it == EXP_SIMPLIFIED)
        {
            *it = EXP_UNKNOWN;
        }
    }
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblemBuilder::simplify(int depth)
{
    KDL::Vector p = testPoints[0];

    for (std::vector<poe_term>::reverse_iterator it = poeTerms.rbegin(); it != poeTerms.rend(); ++it)
    {
        int i = std::distance(poeTerms.begin(), it.base()) - 1;
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() == MatrixExponential::ROTATION && liesOnAxis(exp, p))
        {
            *it = EXP_SIMPLIFIED;
        }
        else
        {
            break;
        }
    }

    if (depth < 1)
    {
        return;
    }

    KDL::Vector k = testPoints[1];

    for (std::vector<poe_term>::iterator it = poeTerms.begin(); it != poeTerms.end(); ++it)
    {
        int i = std::distance(poeTerms.begin(), it);
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() == MatrixExponential::ROTATION && liesOnAxis(exp, k))
        {
            *it = EXP_SIMPLIFIED;
        }
        else
        {
            break;
        }
    }
}

// -----------------------------------------------------------------------------

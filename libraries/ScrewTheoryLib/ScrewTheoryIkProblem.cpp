// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkProblem.hpp"

#include <algorithm>
#include <functional>
#include <iterator>
#include <set>

#include <ColorDebug.h>

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

    struct compare_vectors : public std::binary_function<const KDL::Vector &, const KDL::Vector &, bool>
    {
        result_type operator()(first_argument_type lhs, second_argument_type rhs)
        {
            if (KDL::Equal(lhs.x(), rhs.x()))
            {
                if (KDL::Equal(lhs.y(), rhs.y()))
                {
                    if (KDL::Equal(lhs.z(), rhs.z()))
                    {
                        // treat as equal per !comp(a, b) && !comp(b, a) == true
                        // https://en.cppreference.com/w/cpp/container/set
                        return false;
                    }
                    else if (lhs.z() < rhs.z())
                    {
                        return true;
                    }
                }
                else if (lhs.y() < rhs.y())
                {
                    return true;
                }
            }
            else if (lhs.x() < rhs.x())
            {
                return true;
            }

            return false;
        }
    };

    class poe_term_candidate : public std::unary_function<const ScrewTheoryIkProblemBuilder::PoeTerm &, bool>
    {
    public:
        poe_term_candidate(result_type _expectedKnown)
            : expectedKnown(_expectedKnown),
              expectedSimplified(false),
              ignoreSimplifiedState(true)
        {}

        poe_term_candidate(result_type _expectedKnown, result_type _expectedSimplified)
            : expectedKnown(_expectedKnown),
              expectedSimplified(_expectedSimplified),
              ignoreSimplifiedState(false)
        {}

        result_type operator()(argument_type poeTerm)
        {
            return poeTerm.known == expectedKnown && (ignoreSimplifiedState || poeTerm.simplified == expectedSimplified);
        }

    private:
        result_type expectedKnown, expectedSimplified, ignoreSimplifiedState;
    }
    knownTerm(true), unknownTerm(false), unknownNotSimplifiedTerm(false, false);
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem::ScrewTheoryIkProblem(const PoeExpression & _poe, const std::vector<ScrewTheoryIkSubproblem *> & _steps)
    : poe(_poe),
      steps(_steps)
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

ScrewTheoryIkProblemBuilder::ScrewTheoryIkProblemBuilder(const PoeExpression & _poe)
    : poe(_poe),
      poeTerms(poe.size())
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

    for (int i = 0; i < points.size(); i++)
    {
        CD_DEBUG("points[%d]: [%f %f %f]\n", i, points[i].x(), points[i].y(), points[i].z());
    }
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
                post[j] = post[j] * exp.asFrame(solutions[j](i));
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

    for (int i = poeTerms.size() - 1; i >= 0; i--)
    {
        if (poeTerms[i] == EXP_KNOWN)
        {
            const MatrixExponential & exp = poe.exponentialAtJoint(i);
            H = exp.asFrame(jointValues(i)) * H;
            foundKnown = true;
        }
        else if (foundKnown && poeTerms[i] == EXP_UNKNOWN)
        {
            break;
        }
        else
        {
            continue;
        }
    }

    return H;
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem * ScrewTheoryIkProblem::create(const PoeExpression & poe, const std::vector<ScrewTheoryIkSubproblem *> & steps)
{
    // TODO: validate
    return new ScrewTheoryIkProblem(poe, steps);
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem * ScrewTheoryIkProblemBuilder::build()
{
    // TODO: deallocate memory, implement destructor

    searchPoints();

    testPoints.assign(MAX_SIMPLIFICATION_DEPTH, points[0]);

    std::vector<ScrewTheoryIkSubproblem *> steps;

    std::vector< std::vector<KDL::Vector>::const_iterator > iterators(MAX_SIMPLIFICATION_DEPTH, points.begin());

    int depth = 0;

    do
    {
        for (std::vector<PoeTerm>::iterator it = poeTerms.begin(); it != poeTerms.end(); ++it)
        {
            it->simplified = false;
        }

        simplify(depth);

        ScrewTheoryIkSubproblem * subproblem = trySolve(depth);

        if (subproblem != NULL)
        {
            steps.push_back(subproblem);
            iterators.assign(iterators.size(), points.begin());
            testPoints[0] = points[0];
            depth = 0;
            continue;
        }

        for (int stage = 0; stage < iterators.size(); stage++)
        {
            ++iterators[stage];

            if (iterators[stage] == points.end())
            {
                iterators[stage] = points.begin();
                testPoints[stage] = *iterators[stage];

                if (stage == depth)
                {
                    depth++;
                    break;
                }
            }
            else
            {
                testPoints[stage] = *iterators[stage];
                break;
            }
        }
    }
    while (depth < MAX_SIMPLIFICATION_DEPTH && std::count_if(poeTerms.begin(), poeTerms.end(), unknownTerm) != 0);

    if (std::count_if(poeTerms.begin(), poeTerms.end(), knownTerm) == poe.size())
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

ScrewTheoryIkSubproblem * ScrewTheoryIkProblemBuilder::trySolve(int depth)
{
    int unknownsCount = std::count_if(poeTerms.begin(), poeTerms.end(), unknownNotSimplifiedTerm);

    if (unknownsCount > MAX_SIMPLIFICATION_DEPTH)
    {
        return NULL;
    }

    std::vector<PoeTerm>::reverse_iterator lastUnknown = std::find_if(poeTerms.rbegin(), poeTerms.rend(), unknownNotSimplifiedTerm);
    int lastExpId = std::distance(poeTerms.begin(), lastUnknown.base()) - 1;
    const MatrixExponential & lastExp = poe.exponentialAtJoint(lastExpId);

    if (unknownsCount == 1)
    {
        if (depth == 0)
        {
            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && !liesOnAxis(lastExp, testPoints[0]))
            {
                poeTerms[lastExpId].known = true;
                return new PadenKahanOne(lastExpId, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION)
            {
                poeTerms[lastExpId].known = true;
                return new PardosOne(lastExpId, lastExp, testPoints[0]);
            }
        }

        if (depth == 1)
        {
            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && !liesOnAxis(lastExp, testPoints[0])
                    && !liesOnAxis(lastExp, testPoints[1]))
            {
                poeTerms[lastExpId].known = true;
                return new PadenKahanThree(lastExpId, lastExp, testPoints[0], testPoints[1]);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION)
            {
                poeTerms[lastExpId].known = true;
                return new PardosThree(lastExpId, lastExp, testPoints[0], testPoints[1]);
            }
        }
    }
    else if (unknownsCount == 2 && lastUnknown != poeTerms.rend())
    {
        std::vector<PoeTerm>::reverse_iterator nextToLastUnknown = lastUnknown;
        std::advance(nextToLastUnknown, 1);

        if (!unknownNotSimplifiedTerm(*nextToLastUnknown))
        {
            return NULL;
        }

        int nextToLastExpId = lastExpId - 1;
        const MatrixExponential & nextToLastExp = poe.exponentialAtJoint(nextToLastExpId);

        if (depth == 0)
        {
            KDL::Vector r;

            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && nextToLastExp.getMotionType() == MatrixExponential::ROTATION
                    && !parallelAxes(lastExp, nextToLastExp)
                    && intersectingAxes(lastExp, nextToLastExp, r))
            {
                poeTerms[lastExpId].known = poeTerms[nextToLastExpId].known = true;
                return new PadenKahanTwo(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0], r);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION
                    && nextToLastExp.getMotionType() == MatrixExponential::TRANSLATION
                    && !parallelAxes(lastExp, nextToLastExp)
                    && intersectingAxes(lastExp, nextToLastExp, r))
            {
                poeTerms[lastExpId].known = poeTerms[nextToLastExpId].known = true;
                return new PardosTwo(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && nextToLastExp.getMotionType() == MatrixExponential::ROTATION
                    && parallelAxes(lastExp, nextToLastExp)
                    && !colinearAxes(lastExp, nextToLastExp))
            {
                poeTerms[lastExpId].known = poeTerms[nextToLastExpId].known = true;
                return new PardosFour(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }
        }
    }

    return NULL;
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblemBuilder::simplify(int depth)
{
    KDL::Vector p = testPoints[0];

    for (std::vector<PoeTerm>::reverse_iterator it = poeTerms.rbegin(); it != poeTerms.rend(); ++it)
    {
        int i = std::distance(poeTerms.begin(), it.base()) - 1;
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() == MatrixExponential::ROTATION && liesOnAxis(exp, p))
        {
            it->simplified = true;
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

    for (std::vector<PoeTerm>::iterator it = poeTerms.begin(); it != poeTerms.end(); ++it)
    {
        int i = std::distance(poeTerms.begin(), it);
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() == MatrixExponential::ROTATION && liesOnAxis(exp, k))
        {
            it->simplified = true;
        }
        else
        {
            break;
        }
    }
}

// -----------------------------------------------------------------------------

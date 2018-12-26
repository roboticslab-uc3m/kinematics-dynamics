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

    inline bool perpendicularAxes(const MatrixExponential & exp1, const MatrixExponential & exp2)
    {
        return KDL::Equal(KDL::dot(exp1.getAxis(), exp2.getAxis()), 0);
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

    inline bool planarMovement(const MatrixExponential & exp1, const MatrixExponential & exp2)
    {
        bool sameMotionType = exp1.getMotionType() == exp2.getMotionType();
        return sameMotionType ? parallelAxes(exp1, exp2) : perpendicularAxes(exp1, exp2);
    }

    inline bool normalPlaneMovement(const MatrixExponential & exp1, const MatrixExponential & exp2)
    {
        bool sameMotionType = exp1.getMotionType() == exp2.getMotionType();
        return sameMotionType ? perpendicularAxes(exp1, exp2) : parallelAxes(exp1, exp2);
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
    };

    // Can't inline into previous definition, Doxygen output is messed up by the first variable.
    poe_term_candidate knownTerm(true), unknownTerm(false), unknownNotSimplifiedTerm(false, false);

    void clearSteps(ScrewTheoryIkProblem::Steps & steps)
    {
        for (ScrewTheoryIkProblem::Steps::iterator it = steps.begin(); it != steps.end(); ++it)
        {
            delete (*it);
        }

        steps.clear();
    }
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblemBuilder::ScrewTheoryIkProblemBuilder(const PoeExpression & _poe)
    : poe(_poe),
      poeTerms(poe.size())
{}

// -----------------------------------------------------------------------------

std::vector<KDL::Vector> ScrewTheoryIkProblemBuilder::searchPoints(const PoeExpression & poe)
{
    std::set<KDL::Vector, compare_vectors> set;

    // Add origin.
    set.insert(KDL::Vector::Zero());

    for (int i = 0; i < poe.size(); i++)
    {
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() != MatrixExponential::ROTATION)
        {
            continue;
        }

        // Add some point of this axis.
        set.insert(exp.getOrigin());

        // Find intersection between axes.
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

    // Find one additional random point on each axis.
    for (int i = 0; i < poe.size(); i++)
    {
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() != MatrixExponential::ROTATION)
        {
            continue;
        }

        double factor;
        KDL::Vector randomPointOnAxis;

        do
        {
            KDL::random(factor);
            randomPointOnAxis = exp.getOrigin() + factor * exp.getAxis();
        }
        while (!set.insert(randomPointOnAxis).second);
    }

    // Add TCP.
    set.insert(poe.getTransform().p);

    std::vector<KDL::Vector> points(set.size());
    std::copy(set.begin(), set.end(), points.begin());

    return points;
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem * ScrewTheoryIkProblemBuilder::build()
{
    // Reset state, mark all PoE terms as unknown.
    for (std::vector<PoeTerm>::iterator it = poeTerms.begin(); it != poeTerms.end(); ++it)
    {
        it->known = false;
    }

    // Find solutions, if available.
    ScrewTheoryIkProblem::Steps steps = searchSolutions();

    if (std::count_if(poeTerms.begin(), poeTerms.end(), knownTerm) == poe.size())
    {
        // Instantiate solver class.
        return ScrewTheoryIkProblem::create(poe, steps);
    }
    else
    {
        // Free memory allocations.
        clearSteps(steps);
    }

    // No solution found, try with reversed PoE.
    poe.reverseSelf();

    for (std::vector<PoeTerm>::iterator it = poeTerms.begin(); it != poeTerms.end(); ++it)
    {
        it->known = false;
    }

    steps = searchSolutions();

    if (std::count_if(poeTerms.begin(), poeTerms.end(), knownTerm) == poe.size())
    {
        return ScrewTheoryIkProblem::create(poe, steps, true);
    }
    else
    {
        clearSteps(steps);
    }

    return NULL;
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem::Steps ScrewTheoryIkProblemBuilder::searchSolutions()
{
    points = searchPoints(poe);

    // Shared collection of characteristic points to work with.
    testPoints.assign(MAX_SIMPLIFICATION_DEPTH, points[0]);

    ScrewTheoryIkProblem::Steps steps;

    // Vector of iterators (for iterating more than once over the same collection).
    // Size is number of characteristic points being considered at once.
    std::vector< std::vector<KDL::Vector>::const_iterator > iterators(MAX_SIMPLIFICATION_DEPTH, points.begin());

    // 0: try one point, 1: try two points simultaneously, and so on...
    int depth = 0;

    do
    {
        // Reset simplification state.
        for (std::vector<PoeTerm>::iterator it = poeTerms.begin(); it != poeTerms.end(); ++it)
        {
            it->simplified = false;
        }

        // For the current set of characteristic points, try to simplify the PoE.
        simplify(depth);

        // Find a solution if available.
        ScrewTheoryIkSubproblem * subproblem = trySolve(depth);

        if (subproblem != NULL)
        {
            // Solution found, reset and start again. We'll iterate over the same points, taking
            // into account that some terms are already known.
            steps.push_back(subproblem);
            iterators.assign(iterators.size(), points.begin());
            testPoints[0] = points[0];
            depth = 0;
            continue;
        }

        // Advance iterators at each stage (first point, second point (if used), ...).
        for (int stage = 0; stage < iterators.size(); stage++)
        {
            ++iterators[stage];

            if (iterators[stage] == points.end())
            {
                // We've tried all points for this stage, reset and pick first point again.
                iterators[stage] = points.begin();
                testPoints[stage] = *iterators[stage];

                if (stage == depth)
                {
                    // We are ready to try another point simultaneously, increase "depth" level.
                    depth++;
                    break;
                }
            }
            else
            {
                // Still more points available, quit from here and proceed once more with the outer loop.
                testPoints[stage] = *iterators[stage];
                break;
            }
        }
    }
    // Stop if we can't test more points or all terms are known (solution found).
    while (depth < MAX_SIMPLIFICATION_DEPTH && std::count_if(poeTerms.begin(), poeTerms.end(), unknownTerm) != 0);

    return steps;
}

// -----------------------------------------------------------------------------

ScrewTheoryIkSubproblem * ScrewTheoryIkProblemBuilder::trySolve(int depth)
{
    int unknownsCount = std::count_if(poeTerms.begin(), poeTerms.end(), unknownNotSimplifiedTerm);

    if (unknownsCount == 0 || unknownsCount > 2) // TODO: hardcoded
    {
        // Can't solve yet, too many unknowns or oversimplified.
        return NULL;
    }

    // Find rightmost unknown and not simplified PoE term.
    std::vector<PoeTerm>::reverse_iterator lastUnknown = std::find_if(poeTerms.rbegin(), poeTerms.rend(), unknownNotSimplifiedTerm);
    int lastExpId = std::distance(poeTerms.begin(), lastUnknown.base()) - 1;
    const MatrixExponential & lastExp = poe.exponentialAtJoint(lastExpId);

    // Select the most adequate subproblem, if available.
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
                return new PardosGotorOne(lastExpId, lastExp, testPoints[0]);
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
                return new PardosGotorThree(lastExpId, lastExp, testPoints[0], testPoints[1]);
            }
        }
    }
    else if (unknownsCount == 2 && lastUnknown != poeTerms.rend())
    {
        // Pick the previous PoE term.
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
                    && !parallelAxes(lastExp, nextToLastExp))
            {
                poeTerms[lastExpId].known = poeTerms[nextToLastExpId].known = true;
                return new PardosGotorTwo(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && nextToLastExp.getMotionType() == MatrixExponential::ROTATION
                    && parallelAxes(lastExp, nextToLastExp)
                    && !colinearAxes(lastExp, nextToLastExp))
            {
                poeTerms[lastExpId].known = poeTerms[nextToLastExpId].known = true;
                return new PardosGotorFour(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }
        }
    }

    return NULL;
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblemBuilder::simplify(int depth)
{
    simplifyWithPadenKahanOne(testPoints[0]);

    if (depth == 1)
    {
        simplifyWithPadenKahanThree(testPoints[1]);
    }
    else
    {
        for (int i = 0; i < poe.size(); i++)
        {
            if (poe.exponentialAtJoint(i).getMotionType() == MatrixExponential::TRANSLATION)
            {
                simplifyWithPardosOne();
                break;
            }
        }
    }
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblemBuilder::simplifyWithPadenKahanOne(const KDL::Vector & point)
{
    // Pick first rightmost unknown PoE term.
    std::vector<PoeTerm>::reverse_iterator ritUnknown = std::find_if(poeTerms.rbegin(), poeTerms.rend(), unknownTerm);

    for (std::vector<PoeTerm>::reverse_iterator rit = ritUnknown; rit != poeTerms.rend(); ++rit)
    {
        int i = std::distance(rit, poeTerms.rend()) - 1;
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() == MatrixExponential::ROTATION && liesOnAxis(exp, point))
        {
            rit->simplified = true;
        }
        else
        {
            break;
        }
    }
}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblemBuilder::simplifyWithPadenKahanThree(const KDL::Vector & point)
{
    // Pick first leftmost unknown PoE term.
    std::vector<PoeTerm>::iterator itUnknown = std::find_if(poeTerms.begin(), poeTerms.end(), unknownTerm);

    for (std::vector<PoeTerm>::iterator it = itUnknown; it != poeTerms.end(); ++it)
    {
        int i = std::distance(poeTerms.begin(), it);
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() == MatrixExponential::ROTATION && liesOnAxis(exp, point))
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

void ScrewTheoryIkProblemBuilder::simplifyWithPardosOne()
{
    // Pick first leftmost and rightmost unknown PoE terms.
    std::vector<PoeTerm>::iterator itUnknown = std::find_if(poeTerms.begin(), poeTerms.end(), unknownNotSimplifiedTerm);
    std::vector<PoeTerm>::reverse_iterator ritUnknown = std::find_if(poeTerms.rbegin(), poeTerms.rend(), unknownNotSimplifiedTerm);

    int idStart = std::distance(poeTerms.begin(), itUnknown);
    int idEnd = std::distance(ritUnknown, poeTerms.rend()) - 1;

    if (idStart >= idEnd)
    {
        // Same term or something went wrong (all terms have been already simplified).
        return;
    }

    const MatrixExponential & firstExp = poe.exponentialAtJoint(idStart);
    const MatrixExponential & lastExp = poe.exponentialAtJoint(idEnd);

    if (firstExp.getMotionType() == MatrixExponential::TRANSLATION)
    {
        // Advance from the leftmost PoE term.
        for (int i = idEnd - 1; i >= idStart; i--)
        {
            const MatrixExponential & nextExp = poe.exponentialAtJoint(i + 1);

            // Compare two consecutive PoE terms.
            if (i != idStart)
            {
                const MatrixExponential & currentExp = poe.exponentialAtJoint(i);

                if (planarMovement(currentExp, nextExp))
                {
                    // Might be ultimately simplified, let's find out in the next iterations.
                    continue;
                }
            }
            else if (normalPlaneMovement(firstExp, nextExp))
            {
                // Can simplify everything to the *right* of this PoE term.
                for (int j = idStart + 1; j <= idEnd; j++)
                {
                    poeTerms[j].simplified = true;
                }
            }

            break;
        }
    }
    else if (lastExp.getMotionType() == MatrixExponential::TRANSLATION)
    {
        // Advance from the rightmost PoE term.
        for (int i = idStart + 1; i <= idEnd; i++)
        {
            const MatrixExponential & prevExp = poe.exponentialAtJoint(i - 1);

            if (i != idEnd)
            {
                const MatrixExponential & currentExp = poe.exponentialAtJoint(i);

                if (planarMovement(prevExp, currentExp))
                {
                    continue;
                }
            }
            else if (normalPlaneMovement(prevExp, lastExp))
            {
                // Can simplify everything to the *left* of this PoE term.
                for (int j = idEnd - 1; j >= idStart; j--)
                {
                    poeTerms[j].simplified = true;
                }
            }

            break;
        }
    }
}

// -----------------------------------------------------------------------------

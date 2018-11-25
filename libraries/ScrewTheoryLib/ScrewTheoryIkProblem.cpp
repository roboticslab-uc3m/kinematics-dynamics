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
    }
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblemBuilder::ScrewTheoryIkProblemBuilder(const PoeExpression & _poe)
    : poe(_poe)
{}

// -----------------------------------------------------------------------------

void ScrewTheoryIkProblemBuilder::searchPoints()
{
    std::set<KDL::Vector, compare_vectors> set;

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

    points.resize(set.size());
    std::copy(set.begin(), set.end(), points.begin());
}

// -----------------------------------------------------------------------------

bool ScrewTheoryIkProblem::solve(const KDL::Frame & H_S_T, std::vector<KDL::JntArray> & solutions)
{
    return false;
}

// -----------------------------------------------------------------------------

ScrewTheoryIkProblem * ScrewTheoryIkProblem::create(const std::vector<ScrewTheoryIkSubproblem *> & steps)
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

    testPoints.resize(1);
    unknowns.assign(poe.size(), true);
    simplified.assign(poe.size(), false);

    std::vector<ScrewTheoryIkSubproblem *> steps;

    std::vector<KDL::Vector>::const_iterator pointIt = points.begin();

    do
    {
        testPoints[0] = *pointIt;

        ++pointIt;

        do
        {
            ScrewTheoryIkSubproblem * subProblem = trySolve();

            if (subProblem != NULL)
            {
                steps.push_back(subProblem);
                pointIt = points.begin();
                break;
            }
        }
        while (simplify());
    }
    while (pointIt != points.end() && std::count(unknowns.begin(), unknowns.end(), true) != 0);

    if (true) // TODO
    {
        return ScrewTheoryIkProblem::create(steps);
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
    int unknownsCount = std::count(unknowns.begin(), unknowns.end(), true);

    if (unknownsCount > 2)
    {
        return NULL;
    }

    std::vector<bool>::reverse_iterator lastUnknown = std::find(unknowns.rbegin(), unknowns.rend(), true);
    int lastExpId = std::distance(unknowns.begin(), lastUnknown.base()) - 1;
    const MatrixExponential & lastExp = poe.exponentialAtJoint(lastExpId);

    if (unknownsCount == 1)
    {
        if (testPoints.size() == 1)
        {
            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && !liesOnAxis(lastExp, testPoints[0]))
            {
                unknowns[lastExpId] = false;
                return new PadenKahanOne(lastExpId, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION)
            {
                unknowns[lastExpId] = false;
                return new PardosOne(lastExpId, lastExp, testPoints[0]);
            }
        }

        if (testPoints.size() == 2)
        {
            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && !liesOnAxis(lastExp, testPoints[0])
                    && !liesOnAxis(lastExp, testPoints[1]))
            {
                unknowns[lastExpId] = false;
                return new PadenKahanThree(lastExpId, lastExp, testPoints[0], testPoints[1]);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION)
            {
                unknowns[lastExpId] = false;
                return new PardosThree(lastExpId, lastExp, testPoints[0], testPoints[1]);
            }
        }
    }
    else if (unknownsCount == 2)
    {
        std::vector<bool>::reverse_iterator nextToLastUnknown = std::find(lastUnknown, unknowns.rend(), true);
        int nextToLastExpId = std::distance(unknowns.begin(), nextToLastUnknown.base()) - 1;
        const MatrixExponential & nextToLastExp = poe.exponentialAtJoint(nextToLastExpId);

        if (testPoints.size() == 1)
        {
            KDL::Vector _;

            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && nextToLastExp.getMotionType() == MatrixExponential::ROTATION
                    && !parallelAxes(lastExp, nextToLastExp)
                    && intersectingAxes(lastExp, nextToLastExp, _))
            {
                unknowns[lastExpId] = unknowns[nextToLastExpId] = false;
                return new PadenKahanTwo(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::TRANSLATION
                    && nextToLastExp.getMotionType() == MatrixExponential::TRANSLATION
                    && !parallelAxes(lastExp, nextToLastExp)
                    && intersectingAxes(lastExp, nextToLastExp, _))
            {
                unknowns[lastExpId] = unknowns[nextToLastExpId] = false;
                return new PardosTwo(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }

            if (lastExp.getMotionType() == MatrixExponential::ROTATION
                    && nextToLastExp.getMotionType() == MatrixExponential::ROTATION
                    && parallelAxes(lastExp, nextToLastExp)
                    && !colinearAxes(lastExp, nextToLastExp))
            {
                unknowns[lastExpId] = unknowns[nextToLastExpId] = false;
                return new PardosFour(nextToLastExpId, lastExpId, nextToLastExp, lastExp, testPoints[0]);
            }
        }
    }

    return NULL;
}

// -----------------------------------------------------------------------------


bool ScrewTheoryIkProblemBuilder::simplify()
{
    return true;
}

// -----------------------------------------------------------------------------

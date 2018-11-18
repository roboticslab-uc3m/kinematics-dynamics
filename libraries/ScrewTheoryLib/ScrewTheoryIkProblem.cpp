// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ScrewTheoryIkProblem.hpp"

#include <algorithm>
#include <functional>
#include <set>

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

std::vector<KDL::Vector> ScrewTheoryIkProblemBuilder::searchPoints(const PoeExpression & poe)
{
    std::set<KDL::Vector, compare_vectors> points;

    for (int i = 0; i < poe.size(); i++)
    {
        const MatrixExponential & exp = poe.exponentialAtJoint(i);

        if (exp.getMotionType() != MatrixExponential::ROTATION)
        {
            continue;
        }

        points.insert(exp.getOrigin());

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
                points.insert(p);
            }
        }
    }

    std::vector<KDL::Vector> out(points.size());
    std::copy(points.begin(), points.end(), out.begin());

    return out;
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

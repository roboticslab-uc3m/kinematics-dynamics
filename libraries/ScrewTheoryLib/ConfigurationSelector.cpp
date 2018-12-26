// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ConfigurationSelector.hpp"

#include <kdl/utilities/utility.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline bool checkJointInLimits(double q, double qMin, double qMax)
    {
        return q >= (qMin - KDL::epsilon) && q <= (qMax + KDL::epsilon);
    }
}

// -----------------------------------------------------------------------------

bool ConfigurationSelector::configure(const std::vector<KDL::JntArray> & solutions)
{
    configs.resize(solutions.size());

    bool allValid = true;

    for (int i = 0; i < solutions.size(); i++)
    {
        configs[i].store(&solutions[i]);
        configs[i].validate(_qMin, _qMax);
        allValid = allValid && configs[i].isValid();
    }

    return allValid;
}

// -----------------------------------------------------------------------------

void ConfigurationSelector::Configuration::validate(const KDL::JntArray & qMin, const KDL::JntArray & qMax)
{
    valid = true;

    for (int i = 0; i < q->rows(); i++)
    {
        const KDL::JntArray & _q = *q; // avoid calling q->operator()(i)

        if (!checkJointInLimits(_q(i), qMin(i), qMax(i)))
        {
            valid = false;
            break;
        }
    }
}

// -----------------------------------------------------------------------------

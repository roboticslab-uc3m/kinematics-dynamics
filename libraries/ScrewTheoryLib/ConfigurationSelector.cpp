// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ConfigurationSelector.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool ConfigurationSelector::configure(const std::vector<KDL::JntArray> & solutions)
{
    configs.resize(solutions.size());

    bool anyValid = false;

    for (int i = 0; i < solutions.size(); i++)
    {
        configs[i].store(&solutions[i]);
        anyValid |= validate(configs[i]);
    }

    return anyValid;
}

// -----------------------------------------------------------------------------

bool ConfigurationSelector::validate(Configuration & config)
{
    const auto * q = config.retrievePose();

    for (int i = 0; i < q->rows(); i++)
    {
        const auto & _q = *q; // avoid calling q->operator()(i)

        if (!checkJointInLimits(_q(i), _qMin(i), _qMax(i)))
        {
            config.invalidate();
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

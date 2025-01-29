// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ConfigurationSelector.hpp"

#include <cmath> // std::abs

#include <kdl/utilities/utility.h>

using namespace roboticslab;

bool ConfigurationSelectorHumanoidGait::findOptimalConfiguration(const KDL::JntArray & qGuess)
{
    if (qGuess.rows() != 6)
    {
        return false;
    }

    for (int i = 0; i < configs.size(); i++)
    {
        if (configs[i].isValid() && !applyConstraints(configs[i]))
        {
            configs[i].invalidate();
        }
    }

    return ConfigurationSelectorLeastOverallAngularDisplacement::findOptimalConfiguration(qGuess);
}

bool ConfigurationSelectorHumanoidGait::applyConstraints(const Configuration & config)
{
    const auto & q = *config.retrievePose();

    if (std::abs(q(0)) > KDL::PI_2)
    {
        return false;
    }

    if (std::abs(q(1)) > KDL::PI_2)
    {
        return false;
    }

    if (std::abs(q(2)) > KDL::PI_2)
    {
        return false;
    }

    if (q(3) < 0.0)
    {
        return false;
    }

    if (std::abs(q(4)) > KDL::PI_2)
    {
        return false;
    }

    if (std::abs(q(5)) > KDL::PI_2)
    {
        return false;
    }

    return true;
}

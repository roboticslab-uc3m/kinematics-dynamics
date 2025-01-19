// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ConfigurationSelector.hpp"

#include <cmath> // std::abs
#include <numeric> // std::accumulate
#include <set>
#include <utility> // std::pair

using namespace roboticslab;

bool ConfigurationSelectorLeastOverallAngularDisplacement::findOptimalConfiguration(const KDL::JntArray & qGuess)
{
    if (lastValid != INVALID_CONFIG)
    {
        // either keep last valid configuration or skip looking (out of reach)
        return configs[lastValid].isValid();
    }

    // best for all revolute/prismatic joints
    std::set<std::pair<double, int>> displacementPerConfiguration;

    for (int i = 0; i < configs.size(); i++)
    {
        if (configs[i].isValid())
        {
            const auto diffs = getDiffs(qGuess, configs[i]);
            double sum = std::accumulate(diffs.begin(), diffs.end(), 0.0);
            displacementPerConfiguration.emplace(sum, i);
        }
    }

    if (displacementPerConfiguration.empty())
    {
        // no valid configuration found
        return false;
    }

    // std::map keys are sorted, pick std::pair with lowest key (angle sum)
    auto it = displacementPerConfiguration.begin();
    lastValid = it->second;

    return true;
}

std::vector<double> ConfigurationSelectorLeastOverallAngularDisplacement::getDiffs(const KDL::JntArray & qGuess, const Configuration & config)
{
    std::vector<double> diffs;
    diffs.reserve(qGuess.rows());

    for (int i = 0; i < qGuess.rows(); i++)
    {
        const auto & q = *config.retrievePose();
        diffs.push_back(std::abs(qGuess(i) - q(i)));
    }

    return diffs;
}

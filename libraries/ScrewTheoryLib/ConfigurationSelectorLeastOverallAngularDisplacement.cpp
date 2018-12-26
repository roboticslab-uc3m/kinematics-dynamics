// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ConfigurationSelector.hpp"

#include <cmath>
#include <numeric> // std::accumulate
#include <set>
#include <utility> // std::pair

using namespace roboticslab;

bool ConfigurationSelectorLeastOverallAngularDisplacement::findOptimalConfiguration(const KDL::JntArray & qGuess)
{
    if (lastValid != INVALID_CONFIG)
    {
        if (configs[lastValid].isValid())
        {
            // keep last valid configuration
            optimalConfig = configs[lastValid];
            return true;
        }
        else
        {
            // out of reach, skip looking for another valid configuration
            return false;
        }
    }

    typedef std::pair<double, int> SumToId;
    typedef std::set<SumToId> SetType;
    SetType displacementPerConfiguration; // best for all revolute/prismatic joints

    for (int i = 0; i < configs.size(); i++)
    {
        if (configs[i].isValid())
        {
            std::vector<double> diffs = getDiffs(qGuess, configs[i]);
            double sum = std::accumulate(diffs.begin(), diffs.end(), 0.0);
            displacementPerConfiguration.insert(SumToId(sum, i));
        }
    }

    if (displacementPerConfiguration.empty())
    {
        // no valid configuration found
        return false;
    }

    // std::map keys are sorted, pick std::pair with lowest key (angle sum)
    SetType::iterator it = displacementPerConfiguration.begin();

    lastValid = it->second;
    optimalConfig = configs[lastValid];

    return true;
}

std::vector<double> ConfigurationSelectorLeastOverallAngularDisplacement::getDiffs(const KDL::JntArray & qGuess,
        const Configuration & config)
{
    std::vector<double> diffs;
    diffs.reserve(qGuess.rows());

    for (int i = 0; i < qGuess.rows(); i++)
    {
        const KDL::JntArray & q = *config.retrievePose();
        double diff = std::abs(qGuess(i) - q(i));
        diffs.push_back(diff);
    }

    return diffs;
}

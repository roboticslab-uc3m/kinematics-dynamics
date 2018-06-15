// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotConfiguration.hpp"

#include <cmath>
#include <map>
#include <numeric>
#include <utility>

#include <ColorDebug.h>

using namespace roboticslab;

bool AsibotConfigurationLeastOverallAngularDisplacement::findOptimalConfiguration(JointsIn qGuess)
{
    typedef std::pair<double, const Pose *> MapPair;
    typedef std::map<MapPair::first_type, MapPair::second_type> MapType;
    MapType angularDisplacementPerConfiguration;

    if (forwardElbowUp.valid)
    {
        std::vector<double> diffs = getDiffs(qGuess, forwardElbowUp);
        double sum = std::accumulate(diffs.begin(), diffs.end(), 0.0);
        angularDisplacementPerConfiguration.insert(MapPair(sum, &forwardElbowUp));
    }

    if (forwardElbowDown.valid)
    {
        std::vector<double> diffs = getDiffs(qGuess, forwardElbowDown);
        double sum = std::accumulate(diffs.begin(), diffs.end(), 0.0);
        angularDisplacementPerConfiguration.insert(MapPair(sum, &forwardElbowDown));
    }

    if (reversedElbowUp.valid)
    {
        std::vector<double> diffs = getDiffs(qGuess, reversedElbowUp);
        double sum = std::accumulate(diffs.begin(), diffs.end(), 0.0);
        angularDisplacementPerConfiguration.insert(MapPair(sum, &reversedElbowUp));
    }

    if (reversedElbowDown.valid)
    {
        std::vector<double> diffs = getDiffs(qGuess, reversedElbowDown);
        double sum = std::accumulate(diffs.begin(), diffs.end(), 0.0);
        angularDisplacementPerConfiguration.insert(MapPair(sum, &reversedElbowDown));
    }

    if (angularDisplacementPerConfiguration.empty())
    {
        CD_WARNING("No valid configuration found.\n");
        return false;
    }

    // std::map keys are sorted, pick std::pair with lowest key (angle sum)
    MapType::iterator it = angularDisplacementPerConfiguration.begin();
    optimalPose = *it->second;

    // compare with second best option if available
    if (angularDisplacementPerConfiguration.size() > 1)
    {
        Pose alternativePose = *(++it)->second;

        // both candidates share same orientation of joint 1
        if (alternativePose._orient == optimalPose._orient)
        {
            std::vector<double> diffsA = getDiffs(qGuess, optimalPose);
            std::vector<double> diffsB = getDiffs(qGuess, alternativePose);

            // alternative pose minimizes angular travel distance for joint 2;
            // if equal, prefer elbow up configuration
            if (diffsB[1] < diffsA[1] || (diffsB[1] == diffsA[1] && alternativePose._elb == Pose::UP))
            {
                optimalPose = alternativePose;
            }
        }
    }

    CD_SUCCESS("%s\n", optimalPose.toString().c_str());

    return true;
}

std::vector<double> AsibotConfigurationLeastOverallAngularDisplacement::getDiffs(JointsIn qGuess, const Pose & pose)
{
    std::vector<double> diffs(5);

    diffs[0] = std::abs(qGuess[0] - pose._q1);
    diffs[1] = std::abs(qGuess[1] - pose._q2);
    diffs[2] = std::abs(qGuess[2] - pose._q3);
    diffs[3] = std::abs(qGuess[3] - pose._q4);
    diffs[4] = std::abs(qGuess[4] - pose._q5);

    return diffs;
}

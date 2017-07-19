// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ASIBOT_CONFIGURATION_HPP__
#define __ASIBOT_CONFIGURATION_HPP__

#include <vector>

namespace roboticslab
{

/**
 * @ingroup AsibotSolver
 * @brief Interface for a robot configuration strategy selector.
 *
 * Designed with ASIBOT's specific case in mind, which entails up to
 * four different configurations depending on initial angles provided.
 */
class AsibotConfiguration
{
public:

    //! @brief Destructor
    virtual ~AsibotConfiguration() {}

    /**
     * @brief Stores initial values for a specific pose.
     *
     * Distinguishes between elbow up and down poses.
     *
     * @param q1 IK solution for joint 1 [deg]
     * @param q2 IK solution for joint 2 [deg]
     * @param q3u IK solution for joint 3 (elbow up) [deg]
     * @param q3d IK solution for joint 3 (elbow down) [deg]
     * @param q4 IK solution for joint 4 [deg]
     * @param q5 IK solution for joint 5 [deg]
     *
     * @return true/false on success/failure
     */
    virtual bool configure(double q1, double q2, double q3u, double q3d, double q4, double q5) = 0;

    /**
     * @brief Analyzes available configurations and selects the optimal one.
     * @return true/false on success/failure
     */
    virtual bool findOptimalConfiguration() = 0;

    /**
     * @brief Queries computed angles for the optimal configuration.
     * @param q vector of joint angles [deg]
     */
    virtual void retrieveAngles(std::vector<double> & q) = 0;
};

/**
 * @ingroup AsibotSolver
 * @brief IK solver configuration strategy selector based on the overall
 * angle displacement of all joints.
 *
 * Selects the configuration that entails the lowest sum of angular
 * displacements across all joints. Upon choosing between elbow up and
 * down alternatives, a small angular travel distance for joint 2 is
 * preferred.
 */
class AsibotConfigurationLeastOverallAngularDisplacement : public AsibotConfiguration
{
public:

    /**
     * @brief Constructor
     * @param qMin vector of minimum joint limits [deg]
     * @param qMax vector of maximum joint limits [deg]
     */
    AsibotConfigurationLeastOverallAngularDisplacement(const std::vector<double> & qMin, const std::vector<double> & qMax);

    virtual bool configure(double q1, double q2, double q3u, double q3d, double q4, double q5);

    virtual bool findOptimalConfiguration();

    virtual void retrieveAngles(std::vector<double> & q);

private:

    const std::vector<double> & _qMax, & _qMin;

    // 'u': elbow up
    // 'd': elbow down
    // 'f': forward (joint 1 not reversed)
    // 'r': reversed (joint 1 = q1 + 180º)

    double _q1, _q1f, _q1r;
    double _q2, _q2uf, _q2df, _q2ur, _q2dr;
    double _q3, _q3uf, _q3df, _q3ur, _q3dr;
    double _q4, _q4f, _q4r;
    double _q5, _q5f, _q5r;
};

}  // namespace roboticslab

#endif  // __ASIBOT_CONFIGURATION_HPP__

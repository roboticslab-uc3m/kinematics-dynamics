// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ASIBOT_CONFIGURATION_HPP__
#define __ASIBOT_CONFIGURATION_HPP__

#include <vector>
#include <string>

namespace roboticslab
{

/**
 * @ingroup AsibotSolver
 * @brief Abstract base class for a robot configuration strategy selector.
 *
 * Designed with ASIBOT's specific case in mind, which entails up to
 * four different configurations depending on initial angles provided.
 */
class AsibotConfiguration
{
public:

    //! @brief Const vector of joint angles (input parameter).
    typedef const std::vector<double> & JointsIn;

    //! @brief Vector of joint angles (output parameter).
    typedef std::vector<double> & JointsOut;

    /**
     * @brief Constructor
     * @param qMin vector of minimum joint limits [deg]
     * @param qMax vector of maximum joint limits [deg]
     */
    AsibotConfiguration(JointsIn qMin, JointsIn qMax)
        : _qMin(qMin), _qMax(qMax)
    {}

    //! @brief Destructor
    virtual ~AsibotConfiguration() {}

    /**
     * @brief Stores initial values for a specific pose.
     *
     * Distinguishes between elbow up and down poses. Make sure that:
     *
     * @code oyP = q2u + q3 + q4u = q2d - q3 + q4d @endcode
     *
     * @param q1 IK solution for joint 1 [deg]
     * @param q2u IK solution for joint 2 (elbow up) [deg]
     * @param q2d IK solution for joint 2 (elbow down) [deg]
     * @param q3 IK solution for joint 3 [deg]
     * @param q4u IK solution for joint 4 (elbow up) [deg]
     * @param q4d IK solution for joint 4 (elbow down) [deg]
     * @param q5 IK solution for joint 5 [deg]
     *
     * @return true/false on success/failure
     */
    virtual bool configure(double q1, double q2u, double q2d, double q3, double q4u, double q4d, double q5);

    /**
     * @brief Analyzes available configurations and selects the optimal one.
     * @param qGuess vector of joint angles for current robot position [deg]
     * @return true/false on success/failure
     */
    virtual bool findOptimalConfiguration(JointsIn qGuess) = 0;

    /**
     * @brief Queries computed angles for the optimal configuration.
     * @param q vector of joint angles [deg]
     */
    virtual void retrieveAngles(JointsOut q) const
    {
        optimalPose.retrieveAngles(q);
    }

protected:

    /**
     * @brief Helper class to store a specific robot configuration.
     */
    struct Pose
    {
        //! @brief Orientation of axis 1 ('forward' or 180º offset).
        enum orientation { FORWARD, REVERSED };

        //! @brief Orientation of axes 2-3 (elbow up/down).
        enum elbow { UP, DOWN };

        //! @brief Constructor
        Pose()
            : _q1(0.0), _q2(0.0), _q3(0.0), _q4(0.0), _q5(0.0), valid(true), _orient(FORWARD), _elb(UP)
        {}

        //! @brief Initializes angle values (in degrees).
        void storeAngles(double q1, double q2, double q3, double q4, double q5, orientation orient, elbow elb);

        //! @brief Checks whether current configuration is reachable.
        bool checkJointsInLimits(JointsIn qMin, JointsIn qMax) const;

        //! @brief Returns stored angles.
        void retrieveAngles(JointsOut q) const;

        //! @brief Serializes stored data.
        std::string toString() const;

        double _q1, _q2, _q3, _q4, _q5;
        bool valid;

        orientation _orient;
        elbow _elb;
    };

    JointsIn _qMin, _qMax;

    Pose optimalPose;

    Pose forwardElbowUp;
    Pose forwardElbowDown;
    Pose reversedElbowUp;
    Pose reversedElbowDown;
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
    AsibotConfigurationLeastOverallAngularDisplacement(JointsIn qMin, JointsIn qMax)
        : AsibotConfiguration(qMin, qMax)
    {}

    virtual bool findOptimalConfiguration(JointsIn qGuess);

private:

    //! @brief Obtains vector of differences between current and desired joint angles [deg].
    std::vector<double> getDiffs(JointsIn qGuess, const Pose & pose);
};

/**
 * @ingroup AsibotSolver
 * @brief Base factory class for AsibotConfiguration.
 *
 * Acts as the base class in the <a href="https://stackoverflow.com/a/1741424">
 * abstract factory pattern</a>, encapsulates individual factories.
 */
class AsibotConfigurationFactory
{
public:

    /**
     * @brief Creates an instance of the concrete class.
     * @return A pointer to the base class of the inheritance tree.
     */
    virtual AsibotConfiguration * create() const = 0;
    virtual ~AsibotConfigurationFactory() {}

protected:

    /**
     * @brief Constructor
     * @param qMin vector of minimum joint limits [deg]
     * @param qMax vector of maximum joint limits [deg]
     */
    AsibotConfigurationFactory(AsibotConfiguration::JointsIn qMin, AsibotConfiguration::JointsIn qMax)
        : _qMin(qMin), _qMax(qMax)
    {}

    AsibotConfiguration::JointsIn _qMin, _qMax;
};

/**
 * @ingroup AsibotSolver
 * @brief Implementation factory class for AsibotConfigurationLeastOverallAngularDisplacement.
 *
 * Implements AsibotConfigurationFactory::create.
 */
class AsibotConfigurationLeastOverallAngularDisplacementFactory : public AsibotConfigurationFactory
{
public:

    /**
     * @brief Constructor
     * @param qMin vector of minimum joint limits [deg]
     * @param qMax vector of maximum joint limits [deg]
     */
    AsibotConfigurationLeastOverallAngularDisplacementFactory(AsibotConfiguration::JointsIn qMin, AsibotConfiguration::JointsIn qMax)
        : AsibotConfigurationFactory(qMin, qMax)
    {}

    virtual AsibotConfiguration * create() const
    {
        return new AsibotConfigurationLeastOverallAngularDisplacement(_qMin, _qMax);
    }
};

}  // namespace roboticslab

#endif  // __ASIBOT_CONFIGURATION_HPP__

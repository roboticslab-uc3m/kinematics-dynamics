// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CONFIGURATION_SELECTOR_HPP__
#define __CONFIGURATION_SELECTOR_HPP__

#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/utilities/utility.h>

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Abstract base class for a robot configuration strategy selector
 */
class ConfigurationSelector
{
public:
    /**
     * @brief Constructor
     *
     * @param qMin Joint array of minimum joint limits.
     * @param qMax Joint array of maximum joint limits.
     */
    ConfigurationSelector(const KDL::JntArray & qMin, const KDL::JntArray & qMax)
        : _qMin(qMin),
          _qMax(qMax)
    {}

    //! @brief Destructor
    virtual ~ConfigurationSelector() = default;

    /**
     * @brief Stores initial values for a specific pose.
     *
     * @param solutions Vector of joint arrays that represent all available
     * (valid or not) robot joint poses.
     *
     * @return True/false on success/failure.
     */
    virtual bool configure(const std::vector<KDL::JntArray> & solutions, const std::vector<bool> & reachability);

    /**
     * @brief Analyzes available configurations and selects the optimal one.
     *
     * @param qGuess Joint array of values for current robot position.
     *
     * @return True/false on success/failure.
     */
    virtual bool findOptimalConfiguration(const KDL::JntArray & qGuess) = 0;

    /**
     * @brief Queries computed joint values for the optimal configuration.
     *
     * @param q Output joint array.
     */
    virtual void retrievePose(KDL::JntArray & q) const
    {
        q = *configs[lastValid].retrievePose();
    }

    /**
     * @brief Retrieves the index of the last valid solution.
     *
     * @return Index of the last valid solution.
     */
    int getValidSolutionIndex() const
    { return lastValid; }

    static constexpr int INVALID_CONFIG = -1;

protected:
    /**
     * @brief Helper class to store a specific robot configuration.
     */
    class Configuration
    {
    public:
        //! @brief Initialize joint values.
        void store(const KDL::JntArray * q)
        { this->q = q; }

        //! @brief Retrieve stored joint values.
        const KDL::JntArray * retrievePose() const
        { return q; }

        //! @brief Whether this configuration is attainable or not.
        bool isValid() const
        { return valid; }

        //! @brief Mark this configuration as invalid.
        void invalidate()
        { valid = false; }

    private:
        const KDL::JntArray * q {nullptr};
        bool valid {true};
    };

    /**
     * @brief Validates a specific robot configuration.
     *
     * @param config Configuration to validate.
     *
     * @return True/false on valid/invalid.
     */
    virtual bool validate(Configuration & config);

    /**
     * @brief Checks if a joint value is within its limits.
     *
     * @param q Joint value.
     * @param qMin Minimum joint limit.
     * @param qMax Maximum joint limit.
     *
     * @return True/false on within/without limits.
     */
    static bool checkJointInLimits(double q, double qMin, double qMax)
    {
        return q >= (qMin - KDL::epsilon) && q <= (qMax + KDL::epsilon);
    }

    KDL::JntArray _qMin, _qMax;

    std::vector<Configuration> configs;
    int lastValid {INVALID_CONFIG};
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief IK solver configuration strategy selector based on the overall
 * displacement of all joints.
 *
 * Selects the configuration that entails the lowest sum of displacements
 * across all joints. Works best for all revolute/all prismatic chain types.
 * If attainable, it retains the previous configuration after the first
 * successful choice and discards all other configs for the rest of the
 * instance's lifetime.
 */
class ConfigurationSelectorLeastOverallAngularDisplacement : public ConfigurationSelector
{
public:
    /**
     * @brief Constructor
     *
     * @param qMin Joint array of minimum joint limits.
     * @param qMax Joint array of maximum joint limits.
     */
    ConfigurationSelectorLeastOverallAngularDisplacement(const KDL::JntArray & qMin, const KDL::JntArray & qMax)
        : ConfigurationSelector(qMin, qMax)
    {}

    bool findOptimalConfiguration(const KDL::JntArray & qGuess) override;

protected:
    //! @brief Obtains vector of differences between current and desired joint values.
    std::vector<double> getDiffs(const KDL::JntArray & qGuess, const Configuration & config);
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief IK solver configuration strategy selector based on human walking.
 *
 * Intended for use on lower-body limbs, specifically UC3M TEO's 6-DOF legs.
 * Discards non-human-like configurations, such as the inverted knee pose.
 */
class ConfigurationSelectorHumanoidGait : public ConfigurationSelectorLeastOverallAngularDisplacement
{
public:
    /**
     * @brief Constructor
     *
     * @param qMin Joint array of minimum joint limits.
     * @param qMax Joint array of maximum joint limits.
     */
    ConfigurationSelectorHumanoidGait(const KDL::JntArray & qMin, const KDL::JntArray & qMax)
        : ConfigurationSelectorLeastOverallAngularDisplacement(qMin, qMax)
    {}

    bool findOptimalConfiguration(const KDL::JntArray & qGuess) override;

private:
    //! @brief Determines whether the configuration is valid according to this selector's premises.
    bool applyConstraints(const Configuration & config);
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Base factory class for ConfigurationSelector.
 *
 * Acts as the base class in the <a href="https://stackoverflow.com/a/1741424">
 * abstract factory pattern</a>, encapsulates individual factories.
 */
class ConfigurationSelectorFactory
{
public:
    /**
     * @brief Creates an instance of the concrete class.
     *
     * @return A pointer to the base class of the inheritance tree.
     */
    virtual ConfigurationSelector * create() const = 0;
    virtual ~ConfigurationSelectorFactory() = default;

protected:
    /**
     * @brief Constructor
     *
     * @param qMin Joint array of minimum joint limits.
     * @param qMax Joint array of maximum joint limits.
     */
    ConfigurationSelectorFactory(const KDL::JntArray & qMin, const KDL::JntArray & qMax)
        : _qMin(qMin), _qMax(qMax)
    {}

    KDL::JntArray _qMin, _qMax;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Implementation factory class for ConfigurationSelectorLeastOverallAngularDisplacement.
 *
 * Implements ConfigurationSelectorFactory::create.
 */
class ConfigurationSelectorLeastOverallAngularDisplacementFactory : public ConfigurationSelectorFactory
{
public:
    /**
     * @brief Constructor
     *
     * @param qMin Joint array of minimum joint limits.
     * @param qMax Joint array of maximum joint limits.
     */
    ConfigurationSelectorLeastOverallAngularDisplacementFactory(const KDL::JntArray & qMin, const KDL::JntArray & qMax)
        : ConfigurationSelectorFactory(qMin, qMax)
    {}

    ConfigurationSelector * create() const override
    {
        return new ConfigurationSelectorLeastOverallAngularDisplacement(_qMin, _qMax);
    }
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Implementation factory class for ConfigurationSelectorHumanoidGait.
 *
 * Implements ConfigurationSelectorFactory::create.
 */
class ConfigurationSelectorHumanoidGaitFactory : public ConfigurationSelectorFactory
{
public:
    /**
     * @brief Constructor
     *
     * @param qMin Joint array of minimum joint limits.
     * @param qMax Joint array of maximum joint limits.
     */
    ConfigurationSelectorHumanoidGaitFactory(const KDL::JntArray & qMin, const KDL::JntArray & qMax)
        : ConfigurationSelectorFactory(qMin, qMax)
    {}

    ConfigurationSelector * create() const override
    {
        if (_qMin.rows() == 6 && _qMax.rows() == 6)
        {
            return new ConfigurationSelectorHumanoidGait(_qMin, _qMax);
        }
        else
        {
            return nullptr;
        }
    }
};

} // namespace roboticslab

#endif // __CONFIGURATION_SELECTOR_HPP__

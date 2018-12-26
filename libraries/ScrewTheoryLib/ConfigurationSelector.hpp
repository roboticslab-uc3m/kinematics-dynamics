// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CONFIGURATION_SELECTOR_HPP__
#define __CONFIGURATION_SELECTOR_HPP__

#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

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
    virtual ~ConfigurationSelector() {}

    /**
     * @brief Stores initial values for a specific pose.
     *
     * @param solutions Vector of joint arrays that represent all available
     * (valid or not) robot joint poses.
     *
     * @return True/false on success/failure.
     */
    virtual bool configure(const std::vector<KDL::JntArray> & solutions);

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
        q = *optimalConfig.retrievePose();
    }

protected:

    /**
     * @brief Helper class to store a specific robot configuration.
     */
    class Configuration
    {
    public:

        //! @brief Constructor
        Configuration()
            : q(NULL),
              valid(false)
        {}

        //! @brief Initializes joint values.
        void store(const KDL::JntArray * q)
        { this->q = q; }

        //! @brief Checks reachability against provided joint limits.
        void validate(const KDL::JntArray & qMin, const KDL::JntArray & qMax);

        //! @brief Retrieves stored joint values.
        const KDL::JntArray * retrievePose() const
        { return q; }

        //! @brief Whether this configuration is attainable or not.
        bool isValid() const
        { return valid; }

    private:

        const KDL::JntArray * q;
        bool valid;
    };

    KDL::JntArray _qMin, _qMax;

    Configuration optimalConfig;

    std::vector<Configuration> configs;
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
        : ConfigurationSelector(qMin, qMax),
          lastValid(INVALID_CONFIG)
    {}

    virtual bool findOptimalConfiguration(const KDL::JntArray & qGuess);

private:

    //! @brief Obtains vector of differences between current and desired joint values.
    std::vector<double> getDiffs(const KDL::JntArray & qGuess, const Configuration & config);

    int lastValid;

    static const int INVALID_CONFIG = -1;
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
    virtual ~ConfigurationSelectorFactory() {}

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
        : ConfigurationSelectorFactory( qMin, qMax)
    {}

    virtual ConfigurationSelector * create() const
    {
        return new ConfigurationSelectorLeastOverallAngularDisplacement(_qMin, _qMax);
    }
};

}  // namespace roboticslab

#endif  // __CONFIGURATION_SELECTOR_HPP__

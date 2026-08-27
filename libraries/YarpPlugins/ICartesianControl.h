// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_CONTROL__
#define __I_CARTESIAN_CONTROL__

#include <algorithm> // std::all_of
#include <map>
#include <vector>

#include <yarp/os/Vocab.h>
#include <yarp/dev/ReturnValue.h>

#include <ICartesianSolver.h>

namespace roboticslab
{

/**
 * @brief Abstract base class for a cartesian controller.
 */
class ICartesianControl
{
public:
    /**
     * @brief General-purpose vocabs
     *
     * Used in acknowledge responses, @ref ICartesianControl::Config_commands "configuration accessors", etc..
     */
    enum class Vocabs
    {
        OK = yarp::os::createVocab32('o','k'),             ///< Success
        FAILED = yarp::os::createVocab32('f','a','i','l'), ///< Failure
        SET = yarp::os::createVocab32('s','e','t'),        ///< Setter
        GET = yarp::os::createVocab32('g','e','t'),        ///< Getter
        NOT_SET = yarp::os::createVocab32('n','s','e','t') ///< State: not set
    };

    /**
     * @brief RPC vocabs
     *
     * Used by @ref ICartesianControl_RPC_commands "RPC commands".
     */
    enum class RPC
    {
        STATE = yarp::os::createVocab32('s','t','a','t'), ///< Current state and position
        INV = yarp::os::createVocab32('i','n','v'),       ///< Inverse kinematics
        MOVEJ = yarp::os::createVocab32('m','o','v','j'), ///< Move in joint space, absolute coordinates
        MOVEL = yarp::os::createVocab32('m','o','v','l'), ///< Linear move to target position
        MOVEV = yarp::os::createVocab32('m','o','v','v'), ///< Linear move with given velocity
        GCMP = yarp::os::createVocab32('g','c','m','p'),  ///< Gravity compensation
        FORCE = yarp::os::createVocab32('f','o','r','c'), ///< Force control
        STOP = yarp::os::createVocab32('s','t','o','p'),  ///< Stop control
        TOOL = yarp::os::createVocab32('t','o','o','l'),  ///< Change tool
        ACT = yarp::os::createVocab32('a','c','t')        ///< Actuate tool
    };

    /**
     * @brief Streaming vocabs
     *
     * Used by @ref ICartesianControl_streaming_commands "streaming commands".
     */
    enum class Streaming
    {
        POSE = yarp::os::createVocab32('p','o','s','e'),  ///< Achieve pose
        TWIST = yarp::os::createVocab32('t','w','s','t'), ///< Instantaneous velocity steps
        WRENCH = yarp::os::createVocab32('w','r','n','c') ///< Exert force
    };

    /**
     * @brief Controller mode vocabs
     *
     * Used by ICartesianControl::getState to reflect current control mode.
     */
    enum class Mode
    {
        NONE = yarp::os::createVocab32('n','c','t','l'),  ///< Not controlling
        MOVEJ = yarp::os::createVocab32('m','o','v','j'), ///< Executing MOVEJ command
        MOVEL = yarp::os::createVocab32('m','o','v','l'), ///< Executing MOVEL command
        MOVEV = yarp::os::createVocab32('m','o','v','v'), ///< Executing MOVEV command
        GCMP = yarp::os::createVocab32('g','c','m','p'),  ///< Executing GCMP command
        FORCE = yarp::os::createVocab32('f','o','r','c')  ///< Executing FORCE command
    };

    /**
     * @brief Actuator control vocabs
     *
     * Used by ICartesianControl::act to control the actuator.
     */
    enum class Actuator
    {
        NONE = yarp::os::createVocab32('a','c','n'),      ///< No actuator or no action
        CLOSE = yarp::os::createVocab32('a','c','c','g'), ///< Close gripper
        OPEN = yarp::os::createVocab32('a','c','o','g'),  ///< Open gripper
        STOP = yarp::os::createVocab32('a','c','s','g'),  ///< Stop gripper
        GENERIC = yarp::os::createVocab32('a','c','g')    ///< Generic actuator
    };

    /**
     * @brief Controller configuration vocabs
     *
     * Used by @ref ICartesianControl_config_commands "configuration accessors".
     */
    enum class Config
    {
        GAIN = yarp::os::createVocab32('c','p','c','g'),          ///< Controller gain
        TRAJ_DURATION = yarp::os::createVocab32('c','p','t','d'), ///< Trajectory duration [s]
        TRAJ_REF_SPD = yarp::os::createVocab32('c','p','t','s'),  ///< Trajectory reference speed [m/s]
        TRAJ_REF_ACC = yarp::os::createVocab32('c','p','t','a'),  ///< Trajectory reference acceleration [m/s^2]
        CMC_PERIOD = yarp::os::createVocab32('c','p','c','p'),    ///< CMC period [s]
        FRAME = yarp::os::createVocab32('c','p','f'),             ///< Reference frame
        STREAMING_CMD = yarp::os::createVocab32('c','p','s','c')  ///< Preset streaming command
    };

    /**
     * @brief Controller state structure
     *
     * Used by ICartesianControl::getState to return the current state of the controller.
     */
    struct ControllerState
    {
        std::vector<double> x; ///< Current pose: translation (3, meters) + rotation (3, radians, scaled axis-angle)
        Mode mode;             ///< Current controller mode
        double timestamp;      ///< Timestamp of the last received pose
        double duration;       ///< Duration of the current trajectory, if any (seconds)
        float progress;        ///< Progress of the current trajectory, if any (0.0 to 1.0)
        bool success;          ///< Whether the last trajectory command was successful
    };

    //! Destructor
    virtual ~ICartesianControl() = default;

    //--------------------- RPC commands ---------------------

    /**
     * @anchor ICartesianControl_RPC_commands
     * @name RPC commands
     *
     * RPC commands with success/failure response.
     *
     * @{
     */

    /**
     * @brief Current state and position
     *
     * Inform on control state, get robot position and perform forward kinematics.
     *
     * @param state Controller state data.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue getState(ControllerState & state) = 0;

    /**
     * @brief Inverse kinematics
     *
     * Perform inverse kinematics (using robot position as initial guess), but do not move.
     *
     * @param xd 6-element vector describing desired position in cartesian space; first
     * three elements denote translation (meters), last three denote rotation in scaled
     * axis-angle representation (radians).
     * @param q Vector describing current position in joint space (meters or degrees).
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue solvePose(const std::vector<double> & xd, std::vector<double> & q) = 0;

    /**
     * @brief Move in joint space
     *
     * Perform inverse kinematics and move to desired position in joint space using absolute
     * coordinates.
     *
     * @param xd 6-element vector describing desired position in cartesian space; first
     * three elements denote translation (meters), last three denote rotation in scaled
     * axis-angle representation (radians).
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue moveJoint(const std::vector<double> & xd) = 0;

    /**
     * @brief Linear move to target position
     *
     * Move to end position along a line trajectory.
     *
     * @param xd 6-element vector describing desired position in cartesian space; first
     * three elements denote translation (meters), last three denote rotation in scaled
     * axis-angle representation (radians).
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue moveLinear(const std::vector<double> & xd) = 0;

    /**
     * @brief Linear move with given velocity
     *
     * Move along a line with constant velocity.
     *
     * @param xdotd 6-element vector describing desired velocity in cartesian space; first
     * three elements denote translational velocity (meters/second), last three denote
     * angular velocity (radians/second).
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue moveVelocity(const std::vector<double> & xdotd) = 0;

    /**
     * @brief Gravity compensation
     *
     * Enable gravity compensation.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue gravityCompensation() = 0;

    /**
     * @brief Force control
     *
     * Apply desired forces in task space.
     *
     * @param fd 6-element vector describing desired force exerted by the TCP in
     * cartesian space; first three elements denote linear force (Newton), last
     * three denote torque (Newton*meters).
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue forceControl(const std::vector<double> & fd) = 0;

    /**
     * @brief Stop control
     *
     * Halt current control loop if any and cease movement.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue stopControl() = 0;

    /**
     * @brief Change tool
     *
     * Unload current tool if any and append new tool frame to the kinematic chain.
     *
     * @param x 6-element vector describing new tool tip with regard to current end-effector
     * frame in cartesian space; first three elements denote translation (meters), last three
     * denote rotation in scaled axis-angle representation (radians).
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue changeTool(const std::vector<double> & x) = 0;

    /**
     * @brief Actuate tool
     *
     * Send control command to actuate the robot's tool, if available.
     *
     * @param command One of the available @ref ICartesianControl::Actuator vocabs.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue actuateTool(Actuator command) = 0;

    /** @} */

    //--------------------- Streaming commands ---------------------

    /**
     * @anchor ICartesianControl_streaming_commands
     * @name Streaming commands
     *
     * High-frequency streaming commands, no acknowledge.
     *
     * @{
     */

    /**
     * @brief Achieve pose
     *
     * Move to desired position instantaneously, no further intermediate calculations are
     * expected other than computing the inverse kinematics.
     *
     * @param x 6-element vector describing desired instantaneous pose in cartesian space;
     * first three elements denote translation (meters), last three denote rotation in scaled
     * axis-angle representation (radians).
     */
    virtual void pose(const std::vector<double> & x) = 0;

    /**
     * @brief Instantaneous velocity steps
     *
     * Move in instantaneous velocity increments.
     *
     * @param xdot 6-element vector describing velocity increments in cartesian space;
     * first three elements denote translational velocity (meters/second), last three
     * denote angular velocity (radians/second).
     */
    virtual void twist(const std::vector<double> & xdot) = 0;

    /**
     * @brief Exert force
     *
     * Make the TCP exert the desired force instantaneously.
     *
     * @param w 6-element vector describing desired force exerted by the TCP in cartesian space;
     * first three elements denote linear force (Newton), last three denote torque (Newton*meters).
     */
    virtual void wrench(const std::vector<double> & w) = 0;

    /** @} */

    //--------------------- Configuration accessors ---------------------

    /**
     * @anchor ICartesianControl_config_commands
     * @name Configuration accessors
     *
     * Configuration setters and getters with success/failure response.
     *
     * @{
     */

    /**
     * @brief Set a configuration parameter.
     *
     * Ask the controller to store or update a parameter of 'double' type.
     *
     * @param vocab YARP-encoded vocab (parameter key).
     * @param value Parameter value encoded as a double.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue setParameter(Config vocab, double value) = 0;

    /**
     * @brief Retrieve a configuration parameter.
     *
     * Ask the controller to retrieve a parameter of 'double' type.
     *
     * @param vocab YARP-encoded vocab (parameter key).
     * @param value Parameter value encoded as a double.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue getParameter(Config vocab, double * value) = 0;

    /**
     * @brief Set multiple configuration parameters.
     *
     * Ask the controller to store or update multiple parameters at once.
     *
     * @param params Dictionary of YARP-encoded vocabs as keys and their values.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue setParameters(const std::map<Config, double> & params) = 0;

    /**
     * @brief Retrieve multiple configuration parameters.
     *
     * Ask the controller to retrieve all available parameters at once.
     *
     * @param params Dictionary of YARP-encoded vocabs as keys and their values.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue getParameters(std::map<Config, double> & params) = 0;

    /** @} */

#ifndef SWIG_PREPROCESSOR_SHOULD_SKIP_THIS
    [[deprecated("use `ICartesianControl::getState` instead")]]
    virtual bool stat(std::vector<double> & x, int * state = nullptr, double * timestamp = nullptr)
    {
        ControllerState controllerState;
        auto ret = getState(controllerState);
        x = controllerState.x;
        if (state != nullptr) *state = static_cast<int>(controllerState.mode);
        if (timestamp != nullptr) *timestamp = controllerState.timestamp;
        return ret;
    }

    [[deprecated("use `ICartesianControl::solvePose` instead")]]
    virtual bool inv(const std::vector<double> & xd, std::vector<double> & q)
    { return solvePose(xd, q); }

    [[deprecated("use `ICartesianControl::moveJoint` instead")]]
    virtual bool movj(const std::vector<double> & xd)
    { return moveJoint(xd); }

    [[deprecated("use `ICartesianControl::getState and moveJoint` instead")]]
    virtual bool relj(const std::vector<double> & xd)
    { return false; }

    [[deprecated("use `ICartesianControl::moveLinear` instead")]]
    virtual bool movl(const std::vector<double> & xd)
    { return moveLinear(xd); }

    [[deprecated("use `ICartesianControl::moveVelocity` instead")]]
    virtual bool movv(const std::vector<double> & xdotd)
    { return moveVelocity(xdotd); }

    [[deprecated("use `ICartesianControl::gravityCompensation` instead")]]
    virtual bool gcmp()
    { return gravityCompensation(); }

    [[deprecated("use `ICartesianControl::forceControl` instead")]]
    virtual bool forc(const std::vector<double> & fd)
    { return forceControl(fd); }

    [[deprecated("use `ICartesianControl::getState` instead")]]
    virtual bool wait(double timeout = 0.0)
    { return false; }

    [[deprecated("use `ICartesianControl::changeTool` instead")]]
    virtual bool tool(const std::vector<double> & x)
    { return changeTool(x); }

    [[deprecated("use `ICartesianControl::Actuator` signature instead")]]
    virtual bool act(int command)
    { return actuateTool(static_cast<Actuator>(command)); }

    [[deprecated("use `ICartesianControl::Config` signature instead")]]
    virtual bool setParameter(int vocab, double value)
    { return setParameter(static_cast<Config>(vocab), value); }

    [[deprecated("use `ICartesianControl::Config` signature instead")]]
    virtual bool getParameter(int vocab, double * value)
    { return getParameter(static_cast<Config>(vocab), value); }

    [[deprecated("use `ICartesianControl::Config` signature instead")]]
    virtual bool setParameters(const std::map<int, double> & params)
    { return std::all_of(params.begin(), params.end(), [this](const auto & kv) { return setParameter(static_cast<Config>(kv.first), kv.second); }); }

    [[deprecated("use `ICartesianControl::Config` signature instead")]]
    virtual bool getParameters(std::map<int, double> & params)
    { return std::all_of(params.begin(), params.end(), [this](auto & kv) { return getParameter(static_cast<Config>(kv.first), &kv.second); }); }
#endif // SWIG_PREPROCESSOR_SHOULD_SKIP_THIS
};

} // namespace roboticslab


#ifndef SWIG_PREPROCESSOR_SHOULD_SKIP_THIS
[[deprecated("use `ICartesianControl::Vocabs::OK` instead")]]
constexpr auto VOCAB_CC_OK = static_cast<int>(roboticslab::ICartesianControl::Vocabs::OK);

[[deprecated("use `ICartesianControl::Vocabs::FAILED` instead")]]
constexpr auto VOCAB_CC_FAILED = static_cast<int>(roboticslab::ICartesianControl::Vocabs::FAILED);

[[deprecated("use `ICartesianControl::Vocabs::SET` instead")]]
constexpr auto VOCAB_CC_SET = static_cast<int>(roboticslab::ICartesianControl::Vocabs::SET);

[[deprecated("use `ICartesianControl::Vocabs::GET` instead")]]
constexpr auto VOCAB_CC_GET = static_cast<int>(roboticslab::ICartesianControl::Vocabs::GET);

[[deprecated("use `ICartesianControl::Vocabs::NOT_SET` instead")]]
constexpr auto VOCAB_CC_NOT_SET = static_cast<int>(roboticslab::ICartesianControl::Vocabs::NOT_SET);

[[deprecated("use `ICartesianControl::RPC::STATE` instead")]]
constexpr auto VOCAB_CC_STAT = static_cast<int>(roboticslab::ICartesianControl::RPC::STATE);

[[deprecated("use `ICartesianControl::RPC::INV` instead")]]
constexpr auto VOCAB_CC_INV = static_cast<int>(roboticslab::ICartesianControl::RPC::INV);

[[deprecated("use `ICartesianControl::RPC::MOVEJ` instead")]]
constexpr auto VOCAB_CC_MOVJ = static_cast<int>(roboticslab::ICartesianControl::RPC::MOVEJ);

[[deprecated("use `ICartesianControl::RPC::MOVJ` instead")]]
constexpr auto VOCAB_CC_RELJ = 0;

[[deprecated("use `ICartesianControl::RPC::MOVEL` instead")]]
constexpr auto VOCAB_CC_MOVL = static_cast<int>(roboticslab::ICartesianControl::RPC::MOVEL);

[[deprecated("use `ICartesianControl::RPC::MOVEV` instead")]]
constexpr auto VOCAB_CC_MOVV = static_cast<int>(roboticslab::ICartesianControl::RPC::MOVEV);

[[deprecated("use `ICartesianControl::RPC::GCMP` instead")]]
constexpr auto VOCAB_CC_GCMP = static_cast<int>(roboticslab::ICartesianControl::RPC::GCMP);

[[deprecated("use `ICartesianControl::RPC::FORCE` instead")]]
constexpr auto VOCAB_CC_FORC = static_cast<int>(roboticslab::ICartesianControl::RPC::FORCE);

[[deprecated("use `ICartesianControl::RPC::STOP` instead")]]
constexpr auto VOCAB_CC_STOP = static_cast<int>(roboticslab::ICartesianControl::RPC::STOP);

[[deprecated("use `ICartesianControl::RPC::STATE` instead")]]
constexpr auto VOCAB_CC_WAIT = 0;

[[deprecated("use `ICartesianControl::RPC::TOOL` instead")]]
constexpr auto VOCAB_CC_TOOL = static_cast<int>(roboticslab::ICartesianControl::RPC::TOOL);

[[deprecated("use `ICartesianControl::RPC::ACT` instead")]]
constexpr auto VOCAB_CC_ACT = static_cast<int>(roboticslab::ICartesianControl::RPC::ACT);

[[deprecated("use `ICartesianControl::Streaming::POSE` instead")]]
constexpr auto VOCAB_CC_POSE = static_cast<int>(roboticslab::ICartesianControl::Streaming::POSE);

[[deprecated("use `ICartesianControl::Streaming::TWIST` instead")]]
constexpr auto VOCAB_CC_TWIST = static_cast<int>(roboticslab::ICartesianControl::Streaming::TWIST);

[[deprecated("use `ICartesianControl::Streaming::WRENCH` instead")]]
constexpr auto VOCAB_CC_WRENCH = static_cast<int>(roboticslab::ICartesianControl::Streaming::WRENCH);

[[deprecated("use `ICartesianControl::State::NONE` instead")]]
constexpr auto VOCAB_CC_NOT_CONTROLLING = static_cast<int>(roboticslab::ICartesianControl::Mode::NONE);

[[deprecated("use `ICartesianControl::State::MOVEJ` instead")]]
constexpr auto VOCAB_CC_MOVJ_CONTROLLING = static_cast<int>(roboticslab::ICartesianControl::Mode::MOVEJ);

[[deprecated("use `ICartesianControl::State::MOVEL` instead")]]
constexpr auto VOCAB_CC_MOVL_CONTROLLING = static_cast<int>(roboticslab::ICartesianControl::Mode::MOVEL);

[[deprecated("use `ICartesianControl::State::MOVEV` instead")]]
constexpr auto VOCAB_CC_MOVV_CONTROLLING = static_cast<int>(roboticslab::ICartesianControl::Mode::MOVEV);

[[deprecated("use `ICartesianControl::State::GCMP` instead")]]
constexpr auto VOCAB_CC_GCMP_CONTROLLING = static_cast<int>(roboticslab::ICartesianControl::Mode::GCMP);

[[deprecated("use `ICartesianControl::State::FORCE` instead")]]
constexpr auto VOCAB_CC_FORC_CONTROLLING = static_cast<int>(roboticslab::ICartesianControl::Mode::FORCE);

[[deprecated("use `ICartesianControl::Actuator::NONE` instead")]]
constexpr auto VOCAB_CC_ACTUATOR_NONE = static_cast<int>(roboticslab::ICartesianControl::Actuator::NONE);

[[deprecated("use `ICartesianControl::Actuator::CLOSE` instead")]]
constexpr auto VOCAB_CC_ACTUATOR_CLOSE_GRIPPER = static_cast<int>(roboticslab::ICartesianControl::Actuator::CLOSE);

[[deprecated("use `ICartesianControl::Actuator::OPEN` instead")]]
constexpr auto VOCAB_CC_ACTUATOR_OPEN_GRIPPER = static_cast<int>(roboticslab::ICartesianControl::Actuator::OPEN);

[[deprecated("use `ICartesianControl::Actuator::STOP` instead")]]
constexpr auto VOCAB_CC_ACTUATOR_STOP_GRIPPER = static_cast<int>(roboticslab::ICartesianControl::Actuator::STOP);

[[deprecated("use `ICartesianControl::Actuator::GENERIC` instead")]]
constexpr auto VOCAB_CC_ACTUATOR_GENERIC = static_cast<int>(roboticslab::ICartesianControl::Actuator::GENERIC);

[[deprecated("unused")]]
constexpr auto VOCAB_CC_CONFIG_PARAMS = 0;

[[deprecated("use `ICartesianControl::Config::GAIN` instead")]]
constexpr auto VOCAB_CC_CONFIG_GAIN = static_cast<int>(roboticslab::ICartesianControl::Config::GAIN);

[[deprecated("use `ICartesianControl::Config::TRAJ_DURATION` instead")]]
constexpr auto VOCAB_CC_CONFIG_TRAJ_DURATION = static_cast<int>(roboticslab::ICartesianControl::Config::TRAJ_DURATION);

[[deprecated("use `ICartesianControl::Config::TRAJ_REF_SPD` instead")]]
constexpr auto VOCAB_CC_CONFIG_TRAJ_REF_SPD = static_cast<int>(roboticslab::ICartesianControl::Config::TRAJ_REF_SPD);

[[deprecated("use `ICartesianControl::Config::TRAJ_REF_ACC` instead")]]
constexpr auto VOCAB_CC_CONFIG_TRAJ_REF_ACC = static_cast<int>(roboticslab::ICartesianControl::Config::TRAJ_REF_ACC);

[[deprecated("use `ICartesianControl::Config::CMC_PERIOD` instead")]]
constexpr auto VOCAB_CC_CONFIG_CMC_PERIOD = static_cast<int>(roboticslab::ICartesianControl::Config::CMC_PERIOD);

[[deprecated("unused")]]
constexpr auto VOCAB_CC_CONFIG_WAIT_PERIOD = 0;

[[deprecated("use `ICartesianControl::Config::FRAME` instead")]]
constexpr auto VOCAB_CC_CONFIG_FRAME = static_cast<int>(roboticslab::ICartesianControl::Config::FRAME);

[[deprecated("use `ICartesianControl::Config::STREAMING_CMD` instead")]]
constexpr auto VOCAB_CC_CONFIG_STREAMING_CMD = static_cast<int>(roboticslab::ICartesianControl::Config::STREAMING_CMD);
#endif // SWIG_PREPROCESSOR_SHOULD_SKIP_THIS

#endif // __I_CARTESIAN_CONTROL__

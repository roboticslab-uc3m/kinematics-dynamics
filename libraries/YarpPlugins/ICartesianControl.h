// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_CONTROL__
#define __I_CARTESIAN_CONTROL__

#include <map>
#include <vector>

#include <yarp/os/Vocab.h>

#include <ICartesianSolver.h>

/**
 * @file
 * @brief Contains roboticslab::ICartesianControl and related vocabs.
 * @ingroup YarpPlugins
 * @{
 */

//---------------------------------------------------------------------------------------------------------------
// KEEP VOCAB LIST AND DOCUMENTATION IN SYNC WITH roboticslab::RpcResponder::makeUsage AT CartesianControlServer/
//-----------------------------------------------------------------------------------------------------------------------------------
// USING `int` INSTEAD OF `yarp::conf::vocab32_t` BECAUSE OF SWIG: https://github.com/roboticslab-uc3m/kinematics-dynamics/issues/180
//-----------------------------------------------------------------------------------------------------------------------------------

/**
 * @name General-purpose vocabs
 *
 * Used in acknowledge responses, @ref ICartesianControl_config_commands "configuration accessors", etc..
 *
 * @{
 */

// General-purpose vocabs
constexpr int VOCAB_CC_OK = yarp::os::createVocab32('o','k');              ///< Success
constexpr int VOCAB_CC_FAILED = yarp::os::createVocab32('f','a','i','l');  ///< Failure
constexpr int VOCAB_CC_SET = yarp::os::createVocab32('s','e','t');         ///< Setter
constexpr int VOCAB_CC_GET = yarp::os::createVocab32('g','e','t');         ///< Getter
constexpr int VOCAB_CC_NOT_SET = yarp::os::createVocab32('n','s','e','t'); ///< State: not set

 /** @} */

/**
 * @name RPC vocabs
 *
 * Used by @ref ICartesianControl_RPC_commands "RPC commands" in roboticslab::ICartesianControl.
 *
 * @{
 */

// RPC commands
constexpr int VOCAB_CC_STAT = yarp::os::createVocab32('s','t','a','t'); ///< Current state and position
constexpr int VOCAB_CC_INV = yarp::os::createVocab32('i','n','v');      ///< Inverse kinematics
constexpr int VOCAB_CC_MOVJ = yarp::os::createVocab32('m','o','v','j'); ///< Move in joint space, absolute coordinates
constexpr int VOCAB_CC_RELJ = yarp::os::createVocab32('r','e','l','j'); ///< Move in joint space, relative coordinates
constexpr int VOCAB_CC_MOVL = yarp::os::createVocab32('m','o','v','l'); ///< Linear move to target position
constexpr int VOCAB_CC_MOVV = yarp::os::createVocab32('m','o','v','v'); ///< Linear move with given velocity
constexpr int VOCAB_CC_GCMP = yarp::os::createVocab32('g','c','m','p'); ///< Gravity compensation
constexpr int VOCAB_CC_FORC = yarp::os::createVocab32('f','o','r','c'); ///< Force control
constexpr int VOCAB_CC_STOP = yarp::os::createVocab32('s','t','o','p'); ///< Stop control
constexpr int VOCAB_CC_WAIT = yarp::os::createVocab32('w','a','i','t'); ///< Wait motion done
constexpr int VOCAB_CC_TOOL = yarp::os::createVocab32('t','o','o','l'); ///< Change tool
constexpr int VOCAB_CC_ACT = yarp::os::createVocab32('a','c','t');      ///< Actuate tool

/** @} */

/**
 * @name Streaming vocabs
 *
 * Used by @ref ICartesianControl_streaming_commands "streaming commands" in roboticslab::ICartesianControl.
 *
 * @{
 */

// Streaming commands
constexpr int VOCAB_CC_POSE = yarp::os::createVocab32('p','o','s','e');   ///< Achieve pose
constexpr int VOCAB_CC_TWIST = yarp::os::createVocab32('t','w','s','t');  ///< Instantaneous velocity steps
constexpr int VOCAB_CC_WRENCH = yarp::os::createVocab32('w','r','n','c'); ///< Exert force

/** @} */

/**
 * @name Control state vocabs
 *
 * Used by roboticslab::ICartesianControl::stat to reflect current control state.
 *
 * @{
 */

// Control state
constexpr int VOCAB_CC_NOT_CONTROLLING = yarp::os::createVocab32('c','c','n','c');  ///< Not controlling
constexpr int VOCAB_CC_MOVJ_CONTROLLING = yarp::os::createVocab32('c','c','j','c'); ///< Controlling MOVJ commands
constexpr int VOCAB_CC_MOVL_CONTROLLING = yarp::os::createVocab32('c','c','l','c'); ///< Controlling MOVL commands
constexpr int VOCAB_CC_MOVV_CONTROLLING = yarp::os::createVocab32('c','c','v','c'); ///< Controlling MOVV commands
constexpr int VOCAB_CC_GCMP_CONTROLLING = yarp::os::createVocab32('c','c','g','c'); ///< Controlling GCMP commands
constexpr int VOCAB_CC_FORC_CONTROLLING = yarp::os::createVocab32('c','c','f','c'); ///< Controlling FORC commands

/** @} */

/**
 * @anchor ICartesianControl_actuator_vocabs
 * @name Actuator control vocabs
 *
 * Used by roboticslab::ICartesianControl::act to control the actuator.
 *
 * @{
 */

// Actuator control
constexpr int VOCAB_CC_ACTUATOR_NONE = yarp::os::createVocab32('a','c','n');              ///< No actuator or no action
constexpr int VOCAB_CC_ACTUATOR_CLOSE_GRIPPER = yarp::os::createVocab32('a','c','c','g'); ///< Close gripper
constexpr int VOCAB_CC_ACTUATOR_OPEN_GRIPPER = yarp::os::createVocab32('a','c','o','g');  ///< Open gripper
constexpr int VOCAB_CC_ACTUATOR_STOP_GRIPPER = yarp::os::createVocab32('a','c','s','g');  ///< Stop gripper
constexpr int VOCAB_CC_ACTUATOR_GENERIC = yarp::os::createVocab32('a','c','g');           ///< Generic actuator

/**
 * @name Controller configuration vocabs
 *
 * Used by @ref ICartesianControl_config_commands "configuration accessors".
 *
 * @{
 */

// Controller configuration (parameter keys)
constexpr int VOCAB_CC_CONFIG_PARAMS = yarp::os::createVocab32('p','r','m','s');        ///< Parameter group
constexpr int VOCAB_CC_CONFIG_GAIN = yarp::os::createVocab32('c','p','c','g');          ///< Controller gain
constexpr int VOCAB_CC_CONFIG_TRAJ_DURATION = yarp::os::createVocab32('c','p','t','d'); ///< Trajectory duration [s]
constexpr int VOCAB_CC_CONFIG_TRAJ_REF_SPD = yarp::os::createVocab32('c','p','t','s');  ///< Trajectory reference speed [m/s]
constexpr int VOCAB_CC_CONFIG_TRAJ_REF_ACC = yarp::os::createVocab32('c','p','t','a');  ///< Trajectory reference acceleration [m/s^2]
constexpr int VOCAB_CC_CONFIG_CMC_PERIOD = yarp::os::createVocab32('c','p','c','p');    ///< CMC period [ms]
constexpr int VOCAB_CC_CONFIG_WAIT_PERIOD = yarp::os::createVocab32('c','p','w','p');   ///< Check period of 'wait' command [ms]
constexpr int VOCAB_CC_CONFIG_FRAME = yarp::os::createVocab32('c','p','f');             ///< Reference frame
constexpr int VOCAB_CC_CONFIG_STREAMING_CMD = yarp::os::createVocab32('c','p','s','c'); ///< Preset streaming command

/** @} */

namespace roboticslab
{

/**
 * @brief Abstract base class for a cartesian controller.
 */
class ICartesianControl
{
public:
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
     * @param x 6-element vector describing current position in cartesian space; first
     * three elements denote translation (meters), last three denote rotation in scaled
     * axis-angle representation (radians).
     * @param state Identifier for a cartesian control vocab.
     * @param timestamp Remote encoder acquisition time.
     *
     * @return true on success, false otherwise
     */
    virtual bool stat(std::vector<double> &x, int * state = nullptr, double * timestamp = nullptr) = 0;

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
    virtual bool inv(const std::vector<double> &xd, std::vector<double> &q) = 0;

    /**
     * @brief Move in joint space, absolute coordinates
     *
     * Perform inverse kinematics and move to desired position in joint space using absolute
     * coordinates.
     *
     * @param xd 6-element vector describing desired position in cartesian space; first
     * three elements denote translation (meters), last three denote rotation in scaled
     * axis-angle representation (radians).
     *
     * @see relj (relative coordinates)
     *
     * @return true on success, false otherwise
     */
    virtual bool movj(const std::vector<double> &xd) = 0;

    /**
     * @brief Move in joint space, relative coordinates
     *
     * Perform inverse kinematics and move to desired position in joint space using relative
     * coordinates.
     *
     * @param xd 6-element vector describing desired offset in cartesian space; first
     * three elements denote translation (meters), last three denote rotation in scaled
     * axis-angle representation (radians).
     *
     * @see movj (absolute coordinates)
     *
     * @return true on success, false otherwise
     */
    virtual bool relj(const std::vector<double> &xd) = 0;

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
    virtual bool movl(const std::vector<double> &xd) = 0;

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
    virtual bool movv(const std::vector<double> &xdotd) = 0;

    /**
     * @brief Gravity compensation
     *
     * Enable gravity compensation.
     *
     * @return true on success, false otherwise
     */
    virtual bool gcmp() = 0;

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
    virtual bool forc(const std::vector<double> &fd) = 0;

    /**
     * @brief Stop control
     *
     * Halt current control loop if any and cease movement.
     *
     * @return true on success, false otherwise
     */
    virtual bool stopControl() = 0;

    /**
     * @brief Wait until completion
     *
     * Block execution until the movement is completed, errors occur or timeout
     * is reached.
     *
     * @param timeout Timeout in seconds, '0.0' means no timeout.
     *
     * @return true on success, false if errors occurred during the execution
     * of the trajectory
     */
    virtual bool wait(double timeout = 0.0) = 0;

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
    virtual bool tool(const std::vector<double> &x) = 0;

    /**
     * @brief Actuate tool
     *
     * Send control command to actuate the robot's tool, if available.
     *
     * @param command One of available @ref ICartesianControl_actuator_vocabs "actuator vocabs".
     *
     * @return true on success, false otherwise
     */
    virtual bool act(int command) = 0;

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
    virtual void pose(const std::vector<double> &x) = 0;

    /**
     * @brief Instantaneous velocity steps
     *
     * Move in instantaneous velocity increments.
     *
     * @param xdot 6-element vector describing velocity increments in cartesian space;
     * first three elements denote translational velocity (meters/second), last three
     * denote angular velocity (radians/second).
     */
    virtual void twist(const std::vector<double> &xdot) = 0;

    /**
     * @brief Exert force
     *
     * Make the TCP exert the desired force instantaneously.
     *
     * @param w 6-element vector describing desired force exerted by the TCP in cartesian space;
     * first three elements denote linear force (Newton), last three denote torque (Newton*meters).
     */
    virtual void wrench(const std::vector<double> &w) = 0;

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
    virtual bool setParameter(int vocab, double value) = 0;

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
    virtual bool getParameter(int vocab, double * value) = 0;

    /**
     * @brief Set multiple configuration parameters.
     *
     * Ask the controller to store or update multiple parameters at once.
     *
     * @param params Dictionary of YARP-encoded vocabs as keys and their values.
     *
     * @return true on success, false otherwise
     */
    virtual bool setParameters(const std::map<int, double> & params) = 0;

    /**
     * @brief Retrieve multiple configuration parameters.
     *
     * Ask the controller to retrieve all available parameters at once.
     *
     * @param params Dictionary of YARP-encoded vocabs as keys and their values.
     *
     * @return true on success, false otherwise
     */
    virtual bool getParameters(std::map<int, double> & params) = 0;

    /** @} */
};

} // namespace roboticslab

/** @} */

#endif // __I_CARTESIAN_CONTROL__

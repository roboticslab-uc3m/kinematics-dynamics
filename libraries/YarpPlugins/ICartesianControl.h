// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_CONTROL__
#define __I_CARTESIAN_CONTROL__

#include <map>
#include <vector>

#include <yarp/os/Vocab.h>
#include <yarp/dev/ReturnValue.h>

#include <ICartesianSolver.h>

//---------------------------------------------------------------------------------------------------------------
// KEEP VOCAB LIST AND DOCUMENTATION IN SYNC WITH roboticslab::RpcResponder::makeUsage AT CartesianControlServer/
// and roboticslab::CartesianControlServerROS2::configureRosParameters AT CartesianControlServerROS2/
//-----------------------------------------------------------------------------------------------------------------------------------

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
        OK = yarp::os::createVocab32('o','k'),              ///< Success
        FAILED = yarp::os::createVocab32('f','a','i','l'),  ///< Failure
        SET = yarp::os::createVocab32('s','e','t'),         ///< Setter
        GET = yarp::os::createVocab32('g','e','t'),         ///< Getter
        NOT_SET = yarp::os::createVocab32('n','s','e','t')  ///< State: not set
    };

    /**
     * @brief RPC vocabs
     *
     * Used by @ref ICartesianControl_RPC_commands "RPC commands".
     */
    enum class RPC
    {
        STAT = yarp::os::createVocab32('s','t','a','t'), ///< Current state and position
        INV = yarp::os::createVocab32('i','n','v'),      ///< Inverse kinematics
        MOVJ = yarp::os::createVocab32('m','o','v','j'), ///< Move in joint space, absolute coordinates
        RELJ = yarp::os::createVocab32('r','e','l','j'), ///< Move in joint space, relative coordinates
        MOVL = yarp::os::createVocab32('m','o','v','l'), ///< Linear move to target position
        MOVV = yarp::os::createVocab32('m','o','v','v'), ///< Linear move with given velocity
        GCMP = yarp::os::createVocab32('g','c','m','p'), ///< Gravity compensation
        FORC = yarp::os::createVocab32('f','o','r','c'), ///< Force control
        STOP = yarp::os::createVocab32('s','t','o','p'), ///< Stop control
        WAIT = yarp::os::createVocab32('w','a','i','t'), ///< Wait motion done
        TOOL = yarp::os::createVocab32('t','o','o','l'), ///< Change tool
        ACT = yarp::os::createVocab32('a','c','t')       ///< Actuate tool
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
     * @brief Control state vocabs
     *
     * Used by ICartesianControl::stat to reflect current control state.
     */
    enum class State
    {
        NONE = yarp::os::createVocab32('n','c','t','l'), ///< Not controlling
        MOVJ = yarp::os::createVocab32('m','o','v','j'), ///< Executing MOVJ command
        MOVL = yarp::os::createVocab32('m','o','v','l'), ///< Executing MOVL command
        MOVV = yarp::os::createVocab32('m','o','v','v'), ///< Executing MOVV command
        GCMP = yarp::os::createVocab32('g','c','m','p'), ///< Executing GCMP command
        FORC = yarp::os::createVocab32('f','o','r','c')  ///< Executing FORC command
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
        PARAMS = yarp::os::createVocab32('p','r','m','s'),        ///< Parameter group
        GAIN = yarp::os::createVocab32('c','p','c','g'),          ///< Controller gain
        TRAJ_DURATION = yarp::os::createVocab32('c','p','t','d'), ///< Trajectory duration [s]
        TRAJ_REF_SPD = yarp::os::createVocab32('c','p','t','s'),  ///< Trajectory reference speed [m/s]
        TRAJ_REF_ACC = yarp::os::createVocab32('c','p','t','a'),  ///< Trajectory reference acceleration [m/s^2]
        CMC_PERIOD = yarp::os::createVocab32('c','p','c','p'),    ///< CMC period [ms]
        WAIT_PERIOD = yarp::os::createVocab32('c','p','w','p'),   ///< Check period of 'wait' command [ms]
        FRAME = yarp::os::createVocab32('c','p','f'),             ///< Reference frame
        STREAMING_CMD = yarp::os::createVocab32('c','p','s','c')  ///< Preset streaming command
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
     * @param x 6-element vector describing current position in cartesian space; first
     * three elements denote translation (meters), last three denote rotation in scaled
     * axis-angle representation (radians).
     * @param state Identifier for a cartesian control vocab.
     * @param timestamp Remote encoder acquisition time.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue stat(std::vector<double> & x, State * state = nullptr, double * timestamp = nullptr) = 0;

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
    virtual yarp::dev::ReturnValue inv(const std::vector<double> & xd, std::vector<double> & q) = 0;

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
    virtual yarp::dev::ReturnValue movj(const std::vector<double> & xd) = 0;

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
    virtual yarp::dev::ReturnValue relj(const std::vector<double> & xd) = 0;

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
    virtual yarp::dev::ReturnValue movl(const std::vector<double> & xd) = 0;

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
    virtual yarp::dev::ReturnValue movv(const std::vector<double> & xdotd) = 0;

    /**
     * @brief Gravity compensation
     *
     * Enable gravity compensation.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue gcmp() = 0;

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
    virtual yarp::dev::ReturnValue forc(const std::vector<double> & fd) = 0;

    /**
     * @brief Stop control
     *
     * Halt current control loop if any and cease movement.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue stopControl() = 0;

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
    virtual yarp::dev::ReturnValue wait(double timeout = 0.0) = 0;

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
    virtual yarp::dev::ReturnValue tool(const std::vector<double> & x) = 0;

    /**
     * @brief Actuate tool
     *
     * Send control command to actuate the robot's tool, if available.
     *
     * @param command One of the available @ref ICartesianControl::Actuator vocabs.
     *
     * @return true on success, false otherwise
     */
    virtual yarp::dev::ReturnValue act(Actuator command) = 0;

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
};

} // namespace roboticslab

#endif // __I_CARTESIAN_CONTROL__

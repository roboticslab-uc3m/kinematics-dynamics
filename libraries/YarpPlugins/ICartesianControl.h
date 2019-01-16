// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_CONTROL__
#define __I_CARTESIAN_CONTROL__

#include <map>
#include <vector>

#include "ICartesianSolver.h"

#ifndef SWIG_PREPROCESSOR_SHOULD_SKIP_THIS
#define ROBOTICSLAB_VOCAB(a,b,c,d) ((((int)(d))<<24)+(((int)(c))<<16)+(((int)(b))<<8)+((int)(a)))
#endif // SWIG_PREPROCESSOR_SHOULD_SKIP_THIS

/**
 * @file
 * @brief Contains roboticslab::ICartesianControl and related vocabs.
 * @ingroup YarpPlugins
 * @{
 */

//-------------------------------------------------------------------
// KEEP VOCAB LIST AND DOCUMENTATION IN SYNC WITH:
// * roboticslab::RpcResponder::makeUsage at CartesianControlServer/
//-------------------------------------------------------------------

/**
 * @name General-purpose vocabs
 *
 * Used in acknowledge responses, @ref ICartesianControl_config_commands "configuration accessors", etc..
 *
 * @{
 */

// General-purpose vocabs
#define VOCAB_CC_OK ROBOTICSLAB_VOCAB('o','k',0,0)         ///< Success
#define VOCAB_CC_FAILED ROBOTICSLAB_VOCAB('f','a','i','l') ///< Failure
#define VOCAB_CC_SET ROBOTICSLAB_VOCAB('s','e','t',0)      ///< Setter
#define VOCAB_CC_GET ROBOTICSLAB_VOCAB('g','e','t',0)      ///< Getter

 /** @} */

/**
 * @name RPC vocabs
 *
 * Used by @ref ICartesianControl_RPC_commands "RPC commands" in roboticslab::ICartesianControl.
 *
 * @{
 */

// RPC commands
#define VOCAB_CC_STAT ROBOTICSLAB_VOCAB('s','t','a','t') ///< Current state and position
#define VOCAB_CC_INV ROBOTICSLAB_VOCAB('i','n','v',0)    ///< Inverse kinematics
#define VOCAB_CC_MOVJ ROBOTICSLAB_VOCAB('m','o','v','j') ///< Move in joint space, absolute coordinates
#define VOCAB_CC_RELJ ROBOTICSLAB_VOCAB('r','e','l','j') ///< Move in joint space, relative coordinates
#define VOCAB_CC_MOVL ROBOTICSLAB_VOCAB('m','o','v','l') ///< Linear move to target position
#define VOCAB_CC_MOVV ROBOTICSLAB_VOCAB('m','o','v','v') ///< Linear move with given velocity
#define VOCAB_CC_GCMP ROBOTICSLAB_VOCAB('g','c','m','p') ///< Gravity compensation
#define VOCAB_CC_FORC ROBOTICSLAB_VOCAB('f','o','r','c') ///< Force control
#define VOCAB_CC_STOP ROBOTICSLAB_VOCAB('s','t','o','p') ///< Stop control
#define VOCAB_CC_WAIT ROBOTICSLAB_VOCAB('w','a','i','t') ///< Wait motion done
#define VOCAB_CC_TOOL ROBOTICSLAB_VOCAB('t','o','o','l') ///< Change tool

/** @} */

/**
 * @name Streaming vocabs
 *
 * Used by @ref ICartesianControl_streaming_commands "streaming commands" in roboticslab::ICartesianControl.
 *
 * @{
 */

// Streaming commands
#define VOCAB_CC_TWIST ROBOTICSLAB_VOCAB('t','w','s','t') ///< Instantaneous velocity steps
#define VOCAB_CC_POSE ROBOTICSLAB_VOCAB('p','o','s','e')  ///< Achieve pose (velocity control)
#define VOCAB_CC_MOVI ROBOTICSLAB_VOCAB('m','o','v','i')  ///< Achieve pose (position control)

/** @} */

/**
 * @name Control state vocabs
 *
 * Used by roboticslab::ICartesianControl::stat to reflect current control state.
 *
 * @{
 */

// Control state
#define VOCAB_CC_NOT_CONTROLLING ROBOTICSLAB_VOCAB('c','c','n','c')  ///< Not controlling
#define VOCAB_CC_MOVJ_CONTROLLING ROBOTICSLAB_VOCAB('c','c','j','c') ///< Controlling MOVJ commands
#define VOCAB_CC_MOVL_CONTROLLING ROBOTICSLAB_VOCAB('c','c','l','c') ///< Controlling MOVL commands
#define VOCAB_CC_MOVV_CONTROLLING ROBOTICSLAB_VOCAB('c','c','v','c') ///< Controlling MOVV commands
#define VOCAB_CC_GCMP_CONTROLLING ROBOTICSLAB_VOCAB('c','c','g','c') ///< Controlling GCMP commands
#define VOCAB_CC_FORC_CONTROLLING ROBOTICSLAB_VOCAB('c','c','f','c') ///< Controlling FORC commands

/** @} */

/**
 * @name Controller configuration vocabs
 *
 * Used by @ref ICartesianControl_config_commands "configuration accessors".
 *
 * @{
 */

// Controller configuration (parameter keys)
#define VOCAB_CC_CONFIG_PARAMS ROBOTICSLAB_VOCAB('p','r','m','s')        ///< Parameter group
#define VOCAB_CC_CONFIG_GAIN ROBOTICSLAB_VOCAB('c','p','c','g')          ///< Controller gain
#define VOCAB_CC_CONFIG_MAX_JOINT_VEL ROBOTICSLAB_VOCAB('c','p','j','v') ///< Maximum joint velocity
#define VOCAB_CC_CONFIG_TRAJ_DURATION ROBOTICSLAB_VOCAB('c','p','t','d') ///< Trajectory duration
#define VOCAB_CC_CONFIG_CMC_RATE ROBOTICSLAB_VOCAB('c','p','c','r')      ///< CMC rate [ms]
#define VOCAB_CC_CONFIG_WAIT_PERIOD ROBOTICSLAB_VOCAB('c','p','w','p')   ///< Check period of 'wait' command [ms]
#define VOCAB_CC_CONFIG_FRAME ROBOTICSLAB_VOCAB('c','p','f',0)           ///< Reference frame
#define VOCAB_CC_CONFIG_STREAMING ROBOTICSLAB_VOCAB('c','p','s','c')     ///< Preset streaming command

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
        virtual ~ICartesianControl() {}

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
         * @param state Identifier for a cartesian control vocab.
         * @param x 6-element vector describing current position in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         *
         * @return true on success, false otherwise
         */
        virtual bool stat(int &state, std::vector<double> &x) = 0;

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
         * @param td 6-element vector describing desired forces in cartesian space; first
         * three elements denote translational acceleration (meters/second²), last three
         * denote angular acceleration (radians/second²).
         *
         * @return true on success, false otherwise
         */
        virtual bool forc(const std::vector<double> &td) = 0;

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
         * @brief Achieve pose (velocity control)
         *
         * Move to desired position, computing the error with respect to the current pose. Then,
         * perform numerical differentiation and obtain the final velocity increment (as in @ref twist).
         *
         * @param x 6-element vector describing desired instantaneous pose in cartesian space;
         * first three elements denote translation (meters), last three denote rotation (radians).
         * @param interval Time interval between successive command invocations, expressed in seconds
         * and used for numerical differentiation with desired/current poses.
         */
        virtual void pose(const std::vector<double> &x, double interval) = 0;

        /**
         * @brief Achieve pose (position control)
         *
         * Move to desired position instantaneously, no further intermediate calculations are
         * expected other than computing the inverse kinematics.
         *
         * @param x 6-element vector describing desired instantaneous pose in cartesian space;
         * first three elements denote translation (meters), last three denote rotation (radians).
         */
        virtual void movi(const std::vector<double> &x) = 0;

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

}  // namespace roboticslab

/** @} */

#endif  //  __I_CARTESIAN_CONTROL__

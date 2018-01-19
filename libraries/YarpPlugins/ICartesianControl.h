// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_CONTROL__
#define __I_CARTESIAN_CONTROL__

#include <string>
#include <vector>

#include <yarp/os/Vocab.h>

/**
 * @file
 * @brief Contains roboticslab::ICartesianControl and related vocabs.
 * @ingroup YarpPlugins
 * @{
 */

/**
 * @name RPC vocabs
 *
 * Used by @ref ICartesianControl_RPC_commands "RPC commands" in roboticslab::ICartesianControl.
 *
 * @{
 */

// RPC commands
#define VOCAB_CC_STAT VOCAB4('s','t','a','t') ///< Current state and position
#define VOCAB_CC_INV VOCAB3('i','n','v')      ///< Inverse kinematics
#define VOCAB_CC_MOVJ VOCAB4('m','o','v','j') ///< Move in joint space, absolute coordinates
#define VOCAB_CC_RELJ VOCAB4('r','e','l','j') ///< Move in joint space, relative coordinates
#define VOCAB_CC_MOVL VOCAB4('m','o','v','l') ///< Linear move to target position
#define VOCAB_CC_MOVV VOCAB4('m','o','v','v') ///< Linear move with given velocity
#define VOCAB_CC_GCMP VOCAB4('g','c','m','p') ///< Gravity compensation
#define VOCAB_CC_FORC VOCAB4('f','o','r','c') ///< Force control
#define VOCAB_CC_STOP VOCAB4('s','t','o','p') ///< Stop control
#define VOCAB_CC_TOOL VOCAB4('t','o','o','l') ///< Change tool

/** @} */

/**
 * @name Streaming vocabs
 *
 * Used by @ref ICartesianControl_streaming_commands "streaming commands" in roboticslab::ICartesianControl.
 *
 * @{
 */

// Streaming commands
#define VOCAB_CC_FWD VOCAB3('f','w','d')      ///< Move forward (relative to end-effector)
#define VOCAB_CC_BKWD VOCAB4('b','k','w','d') ///< Move backwards (relative to end-effector)
#define VOCAB_CC_ROT VOCAB3('r','o','t')      ///< Rotate in end-effector frame
#define VOCAB_CC_PAN VOCAB3('p','a','n')      ///< Pan in end-effector frame
#define VOCAB_CC_VMOS VOCAB4('v','m','o','s') ///< Instantaneous velocity steps (inertial frame)
#define VOCAB_CC_EFF VOCAB3('e','f','f')      ///< Instantaneous velocity steps (end-effector frame)
#define VOCAB_CC_POSE VOCAB4('p','o','s','e') ///< Achieve pose in inertial frame

/** @} */

/**
 * @name Control state vocabs
 *
 * Used by roboticslab::ICartesianControl::stat to reflect current control state.
 *
 * @{
 */

// Control state
#define VOCAB_CC_NOT_CONTROLLING VOCAB4('c','c','n','c')  ///< Not controlling
#define VOCAB_CC_MOVJ_CONTROLLING VOCAB4('c','c','j','c') ///< Controlling MOVJ commands
#define VOCAB_CC_MOVL_CONTROLLING VOCAB4('c','c','l','c') ///< Controlling MOVL commands
#define VOCAB_CC_MOVV_CONTROLLING VOCAB4('c','c','v','c') ///< Controlling MOVV commands
#define VOCAB_CC_GCMP_CONTROLLING VOCAB4('c','c','g','c') ///< Controlling GCMP commands
#define VOCAB_CC_FORC_CONTROLLING VOCAB4('c','c','f','c') ///< Controlling FORC commands

/** @} */

/**
 * @name Controller configuration vocabs
 *
 * Used by @ref ICartesianControl_config_commands "Configuration accessors".
 *
 * @{
 */

// Controller configuration (parameter keys)
#define VOCAB_CC_CONFIG_SET_STRING VOCAB4('c','p','s','s')    ///< Set parameter (string)
#define VOCAB_CC_CONFIG_GET_STRING VOCAB4('c','p','g','s')    ///< Get parameter (string)
#define VOCAB_CC_CONFIG_SET_DOUBLE VOCAB4('c','p','s','d')    ///< Set parameter (double)
#define VOCAB_CC_CONFIG_GET_DOUBLE VOCAB4('c','p','g','d')    ///< Get parameter (double)
#define VOCAB_CC_CONFIG_GAIN VOCAB4('c','p','c','g')          ///< Controller gain
#define VOCAB_CC_CONFIG_MAX_JOINT_VEL VOCAB4('c','p','j','v') ///< Maximum joint velocity
#define VOCAB_CC_CONFIG_TRAJ_DURATION VOCAB4('c','p','t','d') ///< Trajectory duration
#define VOCAB_CC_CONFIG_FRAME VOCAB4('c','p','r','f')         ///< Reference frame

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
         * @param q Vector describing current position in joint space (degrees).
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
         * @brief Move forward (relative to end-effector)
         *
         * Move along the Z (positive) axis in velocity increments, applying desired angular
         * velocities. All coordinates are expressed in terms of the end-effector frame.
         * Negative step values will be ignored.
         *
         * @param rot 3-element vector describing desired angular velocity increments in
         * cartesian space, expressed in radians/second.
         * @param step Velocity step corresponding to Z axis, expressed in meters/second.
         *
         * @see bkwd (move backwards)
         * @see eff (rotations + translations)
         */
        virtual void fwd(const std::vector<double> &rot, double step) = 0;

        /**
         * @brief Move backwards (relative to end-effector)
         *
         * Move along the Z (negative) axis in velocity increments, applying desired angular
         * velocities. All coordinates are expressed in terms of the end-effector frame.
         * Negative step values will be ignored.
         *
         * @param 3-element vector describing desired angular velocity increments in
         * cartesian space, expressed in radians/second.
         * @param step Velocity step corresponding to Z axis, expressed in meters/second.
         *
         * @see fwd (move forward)
         * @see eff (rotations + translations)
         */
        virtual void bkwd(const std::vector<double> &rot, double step) = 0;

        /**
         * @brief Rotate in end-effector frame
         *
         * Apply desired angular velocities, but avoid translating, that is, only change the
         * orientation of the TCP. All coordinates are expressed in terms of the end-effector frame.
         *
         * @param 3-element vector describing desired angular velocity increments in
         * cartesian space, expressed in radians/second.
         *
         * @see pan (only translations)
         * @see eff (rotations + translations)
         */
        virtual void rot(const std::vector<double> &rot) = 0;

        /**
         * @brief Pan in end-effector frame
         *
         * Apply desired linear velocities, but avoid rotating, that is, only change the position
         * of the TCP. All coordinates are expressed in terms of the end-effector frame.
         *
         * @param transl 3-element vector describing desired translational velocity
         * increments in cartesian space, expressed in meters/second.
         *
         * @see pan (only rotations)
         * @see eff (rotations + translations)
         */
        virtual void pan(const std::vector<double> &transl) = 0;

        /**
         * @brief Instantaneous velocity steps (inertial frame)
         *
         * Move in instantaneous velocity increments using the fixed (inertial) frame as the
         * reference coordinate system.
         *
         * @param xdot 6-element vector describing velocity increments in cartesian space;
         * first three elements denote translational velocity (meters/second), last three
         * denote angular velocity (radians/second).
         *
         * @see eff (end-effector frame)
         */
        virtual void vmos(const std::vector<double> &xdot) = 0;

        /**
         * @brief Instantaneous velocity steps (end-effector frame)
         *
         * Move in instantaneous velocity increments using the end-effector frame as the
         * reference coordinate system.
         *
         * @param xdotee 6-element vector describing velocity increments in cartesian space;
         * first three elements denote translational velocity (meters/second), last three
         * denote angular velocity (radians/second).
         *
         * @see vmos (inertial frame)
         * @see fwd, bkwd, rot, pan (only translations/rotations)
         */
        virtual void eff(const std::vector<double> &xdotee) = 0;

        /**
         * @brief Achieve pose in inertial frame
         *
         * Move to desired position, computing the error with respect to the current pose. Then,
         * perform numerical differentiation and obtain the final velocity increment (as in @ref vmos).
         *
         * @param x 6-element vector describing desired instantaneous pose in cartesian space;
         * first three elements denote translation (meters), last three denote rotation (radians).
         * @param interval Time interval between successive command invocations, expressed in seconds
         * and used for numerical differentiation with desired/current poses.
         */
        virtual void pose(const std::vector<double> &x, double interval) = 0;

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
         * Ask the controller to store or update a parameter of type std::string.
         *
         * @param vocab YARP-encoded vocab (parameter key).
         * @param value Parameter value encoded as a string.
         *
         * @return true on success, false otherwise
         */
        virtual bool setParameter(int vocab, const std::string & value) = 0;

        /**
         * @brief Retrieve a configuration parameter.
         *
         * Ask the controller to retrieve a parameter of type std::string.
         *
         * @param vocab YARP-encoded vocab (parameter key).
         * @param value Parameter value encoded as a string.
         *
         * @return true on success, false otherwise
         */
        virtual bool getParameter(int vocab, std::string & value) = 0;

        /**
         * @brief Set a configuration parameter.
         *
         * Ask the controller to store or update a parameter of type double.
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
         * Ask the controller to retrieve a parameter of type double.
         *
         * @param vocab YARP-encoded vocab (parameter key).
         * @param value Parameter value encoded as a double.
         *
         * @return true on success, false otherwise
         */
        virtual bool getParameter(int vocab, double * value) = 0;

        /** @} */
};

}  // namespace roboticslab

/** @} */

#endif  //  __I_CARTESIAN_CONTROL__

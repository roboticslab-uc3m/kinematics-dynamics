// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_SOLVER__
#define __I_CARTESIAN_SOLVER__

#include <vector>

#include "roboticslab-kinematics-dynamics-export.h"

#include <ColorDebug.hpp>

/**
 * @file
 * @brief Contains roboticslab::ICartesianSolver
 * @ingroup YarpPlugins
 * @{
 */

namespace roboticslab
{

/**
 * @brief Abstract base class for a cartesian solver.
 */
class ICartesianSolver
{
    public:

        //! Destructor
        virtual ~ICartesianSolver() {}

        /**
         * @deprecated Use @ref getNumJoints instead.
         *
         * @brief Get number of links for which the solver has been configured
         *
         * @param numLinks Number of links.
         *
         * @return true on success, false otherwise
         */
        ROBOTICSLAB_KINEMATICS_DYNAMICS_DEPRECATED virtual bool getNumLinks(int* numLinks)
        {
            CD_WARNING("getNumLinks deprecated: use getNumJoints instead.\n");
            return getNumJoints(numLinks);
        }

        /**
         * @brief Get number of joints for which the solver has been configured
         *
         * @param numJoints Number of joints.
         *
         * @return true on success, false otherwise
         */
        virtual bool getNumJoints(int* numJoints) = 0;

        /**
         * @brief Append an additional link
         *
         * @param x 6-element vector describing end-effector frame in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         *
         * @return true on success, false otherwise
         */
        virtual bool appendLink(const std::vector<double>& x) = 0;

        /**
         * @brief Restore original kinematic chain
         *
         * @return true on success, false otherwise
         */
        virtual bool restoreOriginalChain() = 0;

        /**
         * @brief Perform forward kinematics
         *
         * @param q Vector describing a position in joint space (degrees).
         * @param x 6-element vector describing same position in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         *
         * @return true on success, false otherwise
         */
        virtual bool fwdKin(const std::vector<double> &q, std::vector<double> &x) = 0;

        /**
         * @brief Obtain error with respect to forward kinematics
         *
         * The result is a infinitesimal displacement twist, i.e. a vector, for which the
         * operation of addition makes physical sense.
         *
         * @param xd 6-element vector describing desired position in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param q Vector describing initial position in joint space (degrees).
         * @param x 6-element vector describing the position error in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         *
         * @return true on success, false otherwise
         */
        virtual bool fwdKinError(const std::vector<double> &xd, const std::vector<double> &q, std::vector<double> &x) = 0;

        /**
         * @brief Perform inverse kinematics
         *
         * @param xd 6-element vector describing desired position in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param qGuess Vector describing current position in joint space (degrees).
         * @param q Vector describing target position in joint space (degrees).
         *
         * @return true on success, false otherwise
         */
        virtual bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q) = 0;

        /**
         * @brief Perform differential inverse kinematics
         *
         * @param q Vector describing current position in joint space (degrees).
         * @param xdot 6-element vector describing desired velocity in cartesian space; first
         * three elements denote translational velocity (meters/second), last three denote
         * angular velocity (radians/second).
         * @param qdot Vector describing target velocity in joint space (degrees/second).
         *
         * @return true on success, false otherwise
         */
        virtual bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot) = 0;

        /**
         * @brief Perform differential inverse kinematics on end effector
         *
         * Same as @ref diffInvKin, but the input velocity is expressed in terms of the
         * end effector frame.
         *
         * @param q Vector describing current position in joint space (degrees).
         * @param xdotee 6-element vector describing desired velocity in cartesian space,
         * expressed in the frame of the end effector; first three elements denote translational
         * velocity (meters/second), last three denote angular velocity (radians/second).
         * @param qdot Vector describing target velocity in joint space (degrees/second).
         *
         * @return true on success, false otherwise
         */
        virtual bool diffInvKinEE(const std::vector<double> &q, const std::vector<double> &xdotee, std::vector<double> &qdot) = 0;

        /**
         * @brief Perform inverse dynamics
         *
         * Assumes null joint velocities and accelerations, and no external forces.
         *
         * @param q Vector describing current position in joint space (degrees).
         * @param t 6-element vector describing desired forces in cartesian space; first
         * three elements denote translational acceleration (meters/second²), last three denote
         * angular acceleration (radians/second²).
         *
         * @return true on success, false otherwise
         */
        virtual bool invDyn(const std::vector<double> &q, std::vector<double> &t) = 0;

        /**
         * @brief Perform inverse dynamics
         *
         * @param q Vector describing current position in joint space (degrees).
         * @param qdot Vector describing current velocity in joint space (degrees/second).
         * @param qdotdot Vector describing current acceleration in joint space (degrees/second²).
         * @param fexts vector of external forces applied to each robot segment, expressed in
         * cartesian space; first three elements denote translational acceleration (meters/second²),
         * last three denote angular acceleration (radians/second²).
         * @param t 6-element vector describing desired forces in cartesian space; first
         * three elements denote translational acceleration (meters/second²), last three denote
         * angular acceleration (radians/second²).
         *
         * @return true on success, false otherwise
         */
        virtual bool invDyn(const std::vector<double> &q,const std::vector<double> &qdot, const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t) = 0;

        /**
         * @brief Set joint limits
         *
         * @param qMin Vector of minimum joint values expressed in degrees.
         * @param qMax Vector of maximum joint values expressed in degrees.
         *
         * @return true on success, false otherwise
         */
        virtual bool setLimits(const std::vector<double> &qMin, const std::vector<double> &qMax) = 0;

};

}  // namespace roboticslab

/** @} */

#endif  //  __I_CARTESIAN_SOLVER__

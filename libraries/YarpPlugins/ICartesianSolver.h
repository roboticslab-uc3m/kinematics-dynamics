// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_SOLVER__
#define __I_CARTESIAN_SOLVER__

#include <vector>

#include <yarp/os/Vocab.h>

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

        //! Lists supported reference frames.
        enum reference_frame
        {
            BASE_FRAME = VOCAB4('c','p','f','b'), //!< Base frame
            TCP_FRAME = VOCAB4('c','p','f','t')   //!< End-effector frame (TCP)
        };

        //! Destructor
        virtual ~ICartesianSolver() {}

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
         * @brief Express given pose in other reference frame
         *
         * @param x_in 6-element vector describing a pose in cartesian space, expressed in @p currentFrame;
         * first three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param currentQ Vector describing current robot position in joint space (degrees).
         * @param x_out 6-element vector describing a pose in cartesian space, expressed in @p newFrame;
         * first three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param currentFrame Points at the @ref reference_frame the input pose is expressed in.
         * @param newFrame Points at the @ref reference_frame the output pose should be expressed in.
         *
         * @return true on success, false otherwise
         */
        virtual bool changeReferenceFrame(const std::vector<double> x_in, const std::vector<double> currentQ,
                std::vector<double> x_out, reference_frame currentFrame, reference_frame newFrame) = 0;

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
         * @param frame Points at the @ref reference_frame the desired position is expressed in.
         *
         * @return true on success, false otherwise
         */
        virtual bool fwdKinError(const std::vector<double> &xd, const std::vector<double> &q, std::vector<double> &x, reference_frame frame = BASE_FRAME) = 0;

        /**
         * @brief Perform inverse kinematics
         *
         * @param xd 6-element vector describing desired position in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param qGuess Vector describing current position in joint space (degrees).
         * @param q Vector describing target position in joint space (degrees).
         * @param frame Points at the @ref reference_frame the desired position is expressed in.
         *
         * @return true on success, false otherwise
         */
        virtual bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, reference_frame frame = BASE_FRAME) = 0;

        /**
         * @brief Perform differential inverse kinematics
         *
         * @param q Vector describing current position in joint space (degrees).
         * @param xdot 6-element vector describing desired velocity in cartesian space; first
         * three elements denote translational velocity (meters/second), last three denote
         * angular velocity (radians/second).
         * @param qdot Vector describing target velocity in joint space (degrees/second).
         * @param frame Points at the @ref reference_frame the desired position is expressed in.
         *
         * @return true on success, false otherwise
         */
        virtual bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, reference_frame frame = BASE_FRAME) = 0;

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

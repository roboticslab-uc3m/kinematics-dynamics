// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_SOLVER__
#define __I_CARTESIAN_SOLVER__

#include <vector>

#ifndef SWIG_PREPROCESSOR_SHOULD_SKIP_THIS
#define ROBOTICSLAB_VOCAB(a,b,c,d) ((((int)(d))<<24)+(((int)(c))<<16)+(((int)(b))<<8)+((int)(a)))
#endif // SWIG_PREPROCESSOR_SHOULD_SKIP_THIS

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
            BASE_FRAME = ROBOTICSLAB_VOCAB('c','p','f','b'), //!< Base frame
            TCP_FRAME = ROBOTICSLAB_VOCAB('c','p','f','t')   //!< End-effector frame (TCP)
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
         * @brief Change origin in which a pose is expressed
         *
         * @param x_old_obj_in 6-element vector describing a pose in cartesian space, expressed in the old frame;
         * first three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param x_new_old 6-element vector describing a transformation from the new to the old frame;
         * first three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param x_new_obj 6-element vector describing a pose in cartesian space, expressed in the new frame;
         * first three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         *
         * @return true on success, false otherwise
         */
        virtual bool changeOrigin(const std::vector<double> &x_old_obj,
                                  const std::vector<double> &x_new_old,
                                  std::vector<double> &x_new_obj) = 0;

        /**
         * @brief Perform forward kinematics
         *
         * @param q Vector describing a position in joint space (meters or degrees).
         * @param x 6-element vector describing same position in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         *
         * @return true on success, false otherwise
         */
        virtual bool fwdKin(const std::vector<double> &q, std::vector<double> &x) = 0;

        /**
         * @brief Obtain difference between supplied pose inputs
         *
         * The result is an infinitesimal displacement twist, i.e. a vector, for which the
         * operation of addition makes physical sense.
         *
         * @param xLhs 6-element vector describing a pose in cartesian space (left hand side); first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param xRhs 6-element vector describing a pose in cartesian space (right hand side); first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param xOut 6-element vector describing a pose in cartesian space (result); first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         *
         * @return true on success, false otherwise
         */
        virtual bool poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut) = 0;

        /**
         * @brief Perform inverse kinematics
         *
         * @param xd 6-element vector describing desired position in cartesian space; first
         * three elements denote translation (meters), last three denote rotation in scaled
         * axis-angle representation (radians).
         * @param qGuess Vector describing current position in joint space (meters or degrees).
         * @param q Vector describing target position in joint space (meters or degrees).
         * @param frame Points at the @ref reference_frame the desired position is expressed in.
         *
         * @return true on success, false otherwise
         */
        virtual bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q,
                const reference_frame frame = BASE_FRAME) = 0;

        /**
         * @brief Perform differential inverse kinematics
         *
         * @param q Vector describing current position in joint space (meters or degrees).
         * @param xdot 6-element vector describing desired velocity in cartesian space; first
         * three elements denote translational velocity (meters/second), last three denote
         * angular velocity (radians/second).
         * @param qdot Vector describing target velocity in joint space (meters/second or degrees/second).
         * @param frame Points at the @ref reference_frame the desired position is expressed in.
         *
         * @return true on success, false otherwise
         */
        virtual bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot,
                const reference_frame frame = BASE_FRAME) = 0;

        /**
         * @brief Perform inverse dynamics
         *
         * Assumes null joint velocities and accelerations, and no external forces.
         *
         * @param q Vector describing current position in joint space (meters or degrees).
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
         * @param q Vector describing current position in joint space (meters or degrees).
         * @param qdot Vector describing current velocity in joint space (meters/second or degrees/second).
         * @param qdotdot Vector describing current acceleration in joint space (meters/second² or degrees/second²).
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

};

}  // namespace roboticslab

/** @} */

#endif  //  __I_CARTESIAN_SOLVER__

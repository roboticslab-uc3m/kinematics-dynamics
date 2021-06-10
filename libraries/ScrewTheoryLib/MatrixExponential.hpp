// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MATRIX_EXPONENTIAL_HPP__
#define __MATRIX_EXPONENTIAL_HPP__

#include <kdl/frames.hpp>

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Abstraction of a term in a product of exponentials (POE) formula.
 *
 * @see PoeExpression
 */
class MatrixExponential
{
public:
    //! Lists available screw motion types.
    enum motion
    {
        ROTATION,   ///< Revolute joint (zero-pitch twist).
        TRANSLATION ///< Prismatic joint (infinite-pitch twist).
    };

    /**
     * @brief Constructor
     *
     * @param motionType Screw motion type as defined in \ref motion.
     * @param axis Screw axis.
     * @param origin A point along the screw axis (defaults to (0, 0, 0), ignored in
     * prismatic joints).
     */
    MatrixExponential(motion motionType, const KDL::Vector & axis, const KDL::Vector & origin = KDL::Vector::Zero());

    /**
     * @brief Evaluates this term for the given magnitude of the screw
     *
     * @param theta Input magnitude this screw should be computed at.
     *
     * @return Resulting homogeneous transformation matrix.
     */
    KDL::Frame asFrame(double theta) const;

    /**
     * @brief Retrieves the \ref motion type of this screw
     *
     * @return A \ref motion type this term has been registered with.
     */
    motion getMotionType() const
    { return motionType; }

    /**
     * @brief Screw axis
     *
     * @return A vector representing the direction of the screw axis.
     */
    const KDL::Vector & getAxis() const
    { return axis; }

    /**
     * @brief A point along the screw axis
     *
     * @return A vector representing the position of some point along the screw axis.
     */
    const KDL::Vector & getOrigin() const
    { return origin; }

    /**
     * @brief Refers the internal coordinates of this screw to a different base
     *
     * @param H_new_old Transformation between the new and the old base frame.
     *
     * @see cloneWithBase
     */
    void changeBase(const KDL::Frame & H_new_old);

    /**
     * @brief Clones this instance and refers the internal coordinates of the screw to a different base
     *
     * @param H_new_old Transformation between the new and the old base frame.
     *
     * @return An instance referred to the new frame.
     *
     * @see changeBase
     */
    MatrixExponential cloneWithBase(const KDL::Frame & H_new_old) const;

private:
    motion motionType;
    KDL::Vector axis;
    KDL::Vector origin;
};

} // namespace roboticslab

#endif // __MATRIX_EXPONENTIAL_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PRODUCT_OF_EXPONENTIALS_HPP__
#define __PRODUCT_OF_EXPONENTIALS_HPP__

#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "MatrixExponential.hpp"

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Abstraction of a product of exponentials (POE) formula.
 *
 * This entity is comprised of a sequence of matrix exponentials and a transformation
 * between the base and the tool frame.
 *
 * @see MatrixExponential
 */
class PoeExpression
{
public:
    /**
     * @brief Constructor
     *
     * @param H_S_T Transformation between the base and the tool frame.
     */
    explicit PoeExpression(const KDL::Frame & H_S_T = KDL::Frame::Identity()) : H_S_T(H_S_T) {}

    /**
     * @brief Appends a new term to this POE formula
     *
     * All internal coordinates of the input POE term are referred to the current
     * base frame after applying an (optional) intermediate transformation.
     *
     * @param exp Input POE term.
     * @param H_new_old Transformation between the base frame of this POE and the
     * base frame of the input POE term.
     */
    void append(const MatrixExponential & exp, const KDL::Frame & H_new_old = KDL::Frame::Identity())
    { exps.emplace_back(exp.cloneWithBase(H_new_old)); }

    /**
     * @brief Appends a POE to this formula
     *
     * All internal coordinates of the input POE are referred to the current base
     * frame after applying an (optional) intermediate transformation. The current
     * tool frame is replaced by the appended POE's tool frame and updated in the
     * same manner.
     *
     * @param poe Input POE formula.
     * @param H_new_old Transformation between the new and the old base frame.
     */
    void append(const PoeExpression & poe, const KDL::Frame & H_new_old = KDL::Frame::Identity());

    /**
     * @brief Retrieves the transformation between base and tool frames
     *
     * @return Transformation between the base and the tool frame.
     */
    const KDL::Frame & getTransform() const
    { return H_S_T; }

    /**
     * @brief Size of this POE
     *
     * @return Number of terms in this POE formula.
     */
    int size() const
    { return exps.size(); }

    /**
     * @brief Retrieves a term of the POE formula
     *
     * @param i Zero-based index of the requested term.
     *
     * @return An unmodifiable reference to said term.
     */
    const MatrixExponential & exponentialAtJoint(int i) const
    { return exps.at(i); }

    /**
     * @brief Refers the internal coordinates of this POE to a different base
     *
     * All POE terms as well as the base-to-tool reference frame will be
     * updated accordingly.
     *
     * @param H_new_old Transformation between the new and the old base frame.
     */
    void changeBaseFrame(const KDL::Frame & H_new_old);

    /**
     * @brief Updates the tool frame
     *
     * @param H_new_old Transformation between the new and the old tool frame.
     */
    void changeToolFrame(const KDL::Frame & H_new_old)
    { H_S_T = H_S_T * H_new_old; }

    /**
     * @brief Performs forward kinematics
     *
     * @param q Input joint array (radians).
     * @param H Output pose in cartesian space.
     *
     * @return False if the size of the input joint array does not match the size
     * of this POE.
     */
    bool evaluate(const KDL::JntArray & q, KDL::Frame & H) const;

    /**
     * @brief Inverts this POE formula
     *
     * The sequence of POE terms is reversed and the base and tool frames
     * interchanged. Note that theta_i = -theta_(size-i)_reversed.
     *
     * @see makeReverse
     */
    void reverseSelf();

    /**
     * @brief Creates a new POE entity from the inverse of this POE formula
     *
     * The sequence of POE terms is reversed and the base and tool frames
     * interchanged. Note that theta_i = -theta_(size-i)_reversed.
     *
     * @return A reversed copy of this instance.
     *
     * @see reverseSelf
     */
    PoeExpression makeReverse() const;

    /**
     * @brief Makes a KDL::Chain from this instance
     *
     * @return A KDL kinematic chain representation.
     *
     * @see fromChain
     */
    KDL::Chain toChain() const;

    /**
     * @brief Builds a \ref PoeExpression from a KDL::Chain
     *
     * @param chain A KDL kinematic chain representation.
     *
     * @return The resulting POE formula.
     *
     * @see toChain
     */
    static PoeExpression fromChain(const KDL::Chain & chain);

private:
    std::vector<MatrixExponential> exps;
    KDL::Frame H_S_T;
};

} // namespace roboticslab

#endif // __PRODUCT_OF_EXPONENTIALS_HPP__

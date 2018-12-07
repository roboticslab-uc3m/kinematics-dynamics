// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MATRIX_EXPONENTIAL_HPP__
#define __MATRIX_EXPONENTIAL_HPP__

#include <kdl/frames.hpp>

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class MatrixExponential
{
public:

    enum motion
    {
        ROTATION,
        TRANSLATION
    };

    MatrixExponential(motion motionType, const KDL::Vector & axis, const KDL::Vector & origin = KDL::Vector::Zero());

    KDL::Frame asFrame(double theta) const;

    motion getMotionType() const
    { return motionType; }

    const KDL::Vector & getAxis() const
    { return axis; }

    const KDL::Vector & getOrigin() const
    { return origin; }

    void changeBase(const KDL::Frame & H_new_old);

    MatrixExponential cloneWithBase(const KDL::Frame & H_new_old) const;

private:

    motion motionType;
    KDL::Vector axis;
    KDL::Vector origin;
};

}  // namespace roboticslab

#endif  // __MATRIX_EXPONENTIAL_HPP__

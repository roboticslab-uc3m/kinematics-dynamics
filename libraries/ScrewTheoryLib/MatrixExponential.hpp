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

    MatrixExponential(motion motionType, const KDL::Vector & axis, const KDL::Vector & origin);

    KDL::Frame asFrame(double theta) const;

    motion getMotionType() const
    { return motionType; }

    bool liesOnAxis(const KDL::Vector & point) const;

private:

    motion motionType;
    KDL::Vector axis;
    KDL::Vector origin;
};

}  // namespace roboticslab

#endif  // __MATRIX_EXPONENTIAL_HPP__

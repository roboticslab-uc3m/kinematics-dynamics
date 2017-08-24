// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_VECTOR_CONVERTER_HPP__
#define __KDL_VECTOR_CONVERTER_HPP__

#include <vector>

#include <kdl/frames.hpp>

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-libraries
 * \defgroup KdlVectorConverterLib
 *
 * @brief Contains classes related to kdl and std::vector classes.
 */

/**
 * @ingroup KdlVectorConverterLib
 * @brief Implements related to kdl and std::vector classes.
 */

class KdlVectorConverter
{
public:

    /** Convert from std::vector<double> to KDL::Frame */
    static KDL::Frame vectorToFrame(const std::vector<double> &x);

    /** Convert from KDL::Frame to std::vector<double> */
    static std::vector<double> frameToVector(const KDL::Frame& f);

    /** Convert from std::vector<double> to KDL::Twist */
    static KDL::Twist vectorToTwist(const std::vector<double> &xdot);

    /** Convert from KDL::Twist to std::vector<double> */
    static std::vector<double> twistToVector(const KDL::Twist& t);

private:

    KdlVectorConverter() {}
};

}  // namespace roboticslab

#endif  // __KDL_VECTOR_CONVERTER_HPP__

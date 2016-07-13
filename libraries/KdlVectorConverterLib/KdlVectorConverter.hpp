// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_VECTOR_CONVERTER_HPP__
#define __KDL_VECTOR_CONVERTER_HPP__

#include <vector>

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>

#include "ColorDebug.hpp"

namespace teo
{

/**
 * @ingroup teo_libraries
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

    KdlVectorConverter(std::string angleRepr);

protected:

    bool vectorToFrame(const std::vector<double> &x, KDL::Frame& f);
    bool frameToVector(const KDL::Frame& f, std::vector<double> &x);
    bool twistToVector(const KDL::Twist& t, std::vector<double> &xdot);

    std::string angleRepr;

    /**
    * Simple function to pass from radians to degrees.
    * @param inRad angle value in radians.
    * @return angle value in degrees.
    */
    double toDeg(const double inRad) {
        return (inRad * 180.0 / M_PI);  // return (inRad * 180.0 / 3.14159265);
    }

    /**
    * Simple function to pass from degrees to radians.
    * @param inDeg angle value in degrees.
    * @return angle value in radians.
    */
    double toRad(const double inDeg) {
        return (inDeg * M_PI / 180.0);  // return (inDeg * 3.14159265 / 180.0);
    }
};

}  // namespace teo

#endif  // __KDL_VECTOR_CONVERTER_HPP__

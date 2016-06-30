// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

#include <vector>

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>

#include "ColorDebug.hpp"

#define DEFAULT_MAXVEL 7.5      // unit/s
#define DEFAULT_MAXACC 0.2      // unit/s^2
#define DEFAULT_DURATION 5

namespace teo
{

class Trajectory
{
public:

    Trajectory();

    bool getX(const double movementTime, std::vector<double>& x);

    bool getXdot(const double movementTime, std::vector<double>& xdot);

    bool newLine(const std::vector<double> &src, const std::vector<double> &dest);
    bool deleteLine();

private:

    KDL::Trajectory_Segment* currentTrajectory;
    KDL::RotationalInterpolation_SingleAxis* _orient;

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

#endif  // __TRAJECTORY_HPP__

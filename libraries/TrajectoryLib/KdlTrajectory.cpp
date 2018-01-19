// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlTrajectory.hpp"

#include <kdl/frames.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_line.hpp>

#include <ColorDebug.hpp>

#include "KdlVectorConverter.hpp"

// -----------------------------------------------------------------------------

roboticslab::LineTrajectory::LineTrajectory()
    : currentTrajectory(0),
      _orient(0)
{}

// -----------------------------------------------------------------------------

bool roboticslab::LineTrajectory::getPosition(const double movementTime, std::vector<double>& position)
{
    KDL::Frame xFrame = currentTrajectory->Pos(movementTime);
    position = KdlVectorConverter::frameToVector(xFrame);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LineTrajectory::getVelocity(const double movementTime, std::vector<double>& velocity)
{
    KDL::Twist xdotFrame = currentTrajectory->Vel(movementTime);
    velocity = KdlVectorConverter::twistToVector(xdotFrame);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LineTrajectory::newLine(const std::vector<double> &src, const std::vector<double> &dest)
{
    KDL::Frame srcFrame = KdlVectorConverter::vectorToFrame(src);
    KDL::Frame destFrame = KdlVectorConverter::vectorToFrame(dest);

    _orient = new KDL::RotationalInterpolation_SingleAxis();

    double _eqradius = 1.0; //0.000001;

    KDL::Path * pathLine = new KDL::Path_Line(srcFrame, destFrame, _orient, _eqradius);
    KDL::VelocityProfile * velocityProfile = new KDL::VelocityProfile_Trap(DEFAULT_MAXVEL, DEFAULT_MAXACC);

    currentTrajectory = new KDL::Trajectory_Segment(pathLine, velocityProfile, DEFAULT_DURATION);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LineTrajectory::deleteLine()
{
    delete currentTrajectory;  // deletes _orient, too
    currentTrajectory = 0;
    _orient = 0;

    return true;
}

// -----------------------------------------------------------------------------

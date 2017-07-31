// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LineTrajectory.hpp"

#include <kdl/frames.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_line.hpp>

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

roboticslab::LineTrajectory::LineTrajectory(const std::string & angleRepr)
    : KdlVectorConverter(angleRepr),
      currentTrajectory(0),
      _orient(0)
{}

// -----------------------------------------------------------------------------

bool roboticslab::LineTrajectory::getX(const double movementTime, std::vector<double>& x)
{
    KDL::Frame xFrame = currentTrajectory->Pos(movementTime);
    frameToVector(xFrame,x);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LineTrajectory::getXdot(const double movementTime, std::vector<double>& xdot)
{
    KDL::Twist xdotFrame = currentTrajectory->Vel(movementTime);
    twistToVector(xdotFrame,xdot);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LineTrajectory::newLine(const std::vector<double> &src, const std::vector<double> &dest)
{
    KDL::Frame srcFrame, destFrame;
    vectorToFrame(src,srcFrame);
    vectorToFrame(dest,destFrame);

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

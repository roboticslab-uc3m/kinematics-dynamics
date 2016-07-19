// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Trajectory.hpp"

// -----------------------------------------------------------------------------

teo::LineTrajectory::LineTrajectory() : KdlVectorConverter("axisAngle")
{
    _orient = 0;
    currentTrajectory = 0;
}

// -----------------------------------------------------------------------------

bool teo::LineTrajectory::getX(const double movementTime, std::vector<double>& x)
{
    KDL::Frame xFrame = currentTrajectory->Pos(movementTime);
    frameToVector(xFrame,x);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::LineTrajectory::getXdot(const double movementTime, std::vector<double>& xdot)
{
    KDL::Twist xdotFrame = currentTrajectory->Vel(movementTime);
    twistToVector(xdotFrame,xdot);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::LineTrajectory::newLine(const std::vector<double> &src, const std::vector<double> &dest)
{
    KDL::Frame srcFrame, destFrame;
    vectorToFrame(src,srcFrame);
    vectorToFrame(dest,destFrame);
    _orient = new KDL::RotationalInterpolation_SingleAxis();
    double _eqradius = 1; //0.000001;
    bool _aggregate = true;
    double duration = DEFAULT_DURATION;

    KDL::Path_Line testPathLine(srcFrame, destFrame, _orient, _eqradius, _aggregate);
    KDL::VelocityProfile_Trap testVelocityProfile(DEFAULT_MAXVEL, DEFAULT_MAXACC);
//    KDL::Trajectory_Segment testTrajectory(testPathLine.Clone(), testVelocityProfile.Clone(), duration, _aggregate);
    currentTrajectory = new KDL::Trajectory_Segment(testPathLine.Clone(), testVelocityProfile.Clone(), duration, _aggregate);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LineTrajectory::deleteLine()
{
    delete _orient;
    _orient = 0;
    delete currentTrajectory;
    currentTrajectory = 0;

    return true;
}

// -----------------------------------------------------------------------------

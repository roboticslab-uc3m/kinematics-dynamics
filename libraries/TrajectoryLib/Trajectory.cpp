// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Trajectory.hpp"

// -----------------------------------------------------------------------------

bool teo::Trajectory::getX(const double movementTime, std::vector<double>& desiredX)
{
    //KDL::Frame desiredX = trajectory->Pos(movementTime);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::Trajectory::getXdot(const double movementTime, std::vector<double>& desiredXdot)
{
    //KDL::Twist desiredXdot = trajectory->Vel(movementTime);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::Trajectory::generateLine(const std::vector<double> &src, const std::vector<double> &dest)
{

    _orient = new KDL::RotationalInterpolation_SingleAxis();
    double _eqradius = 1; //0.000001;
    bool _aggregate = true;
    //KDL::Path_Line testPathLine(initF, targetF, _orient, _eqradius, _aggregate);
    //KDL::VelocityProfile_Trap testVelocityProfile(maxVel, maxAcc);
    //currentTrajectory = new Trajectory_Segment(testPathLine.Clone(), testVelocityProfile.Clone(), duration, _aggregate);

    return true;
}

// -----------------------------------------------------------------------------

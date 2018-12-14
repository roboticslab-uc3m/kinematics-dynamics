// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlTrajectory.hpp"

#include <kdl/trajectory_segment.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <ColorDebug.h>

#include "KdlVectorConverter.hpp"

// -----------------------------------------------------------------------------

roboticslab::KdlTrajectory::KdlTrajectory()
    : currentTrajectory(0),
      path(0),
      orient(0),
      velocityProfile(0),
      duration(DURATION_NOT_SET),
      configuredPath(false),
      configuredVelocityProfile(false)
{}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::getDuration(double* duration) const
{
    *duration = currentTrajectory->Duration();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::getPosition(const double movementTime, std::vector<double>& position)
{
    KDL::Frame xFrame = currentTrajectory->Pos(movementTime);
    position = KdlVectorConverter::frameToVector(xFrame);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::getVelocity(const double movementTime, std::vector<double>& velocity)
{
    KDL::Twist xdotFrame = currentTrajectory->Vel(movementTime);
    velocity = KdlVectorConverter::twistToVector(xdotFrame);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::getAcceleration(const double movementTime, std::vector<double>& acceleration)
{
    KDL::Twist xdotdotFrame = currentTrajectory->Acc(movementTime);
    acceleration = KdlVectorConverter::twistToVector(xdotdotFrame);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::setDuration(const double duration)
{
    this->duration = duration;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::addWaypoint(const std::vector<double>& waypoint,
                         const std::vector<double>& waypointVelocity,
                         const std::vector<double>& waypointAcceleration)
{
    KDL::Frame frame = KdlVectorConverter::vectorToFrame(waypoint);
    frames.push_back(frame);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::configurePath(const int pathType)
{
    switch( pathType )
    {
    case ICartesianTrajectory::LINE:
    {
        if ( frames.size() != 2 )
        {
            CD_ERROR("Need exactly 2 waypoints for Cartesian line (have %d)!\n",frames.size());
            return false;
        }

        orient = new KDL::RotationalInterpolation_SingleAxis();
        double eqradius = 1.0; //0.000001;
        path = new KDL::Path_Line(frames[0], frames[1], orient, eqradius);

        configuredPath = true;
        break;
    }
    default:
        CD_ERROR("Only LINE cartesian path implemented for now!\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::configureVelocityProfile(const int velocityProfileType)
{
    switch( velocityProfileType )
    {
    case ICartesianTrajectory::TRAPEZOIDAL:
    {
        velocityProfile = new KDL::VelocityProfile_Trap(DEFAULT_CARTESIAN_MAX_VEL, DEFAULT_CARTESIAN_MAX_ACC);

        configuredVelocityProfile = true;
        break;
    }
    default:
        CD_ERROR("Only TRAPEZOIDAL cartesian velocity profile implemented for now!\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::create()
{
    if( DURATION_NOT_SET == duration )
    {
        CD_ERROR("Duration not set!\n");
        return false;
    }
    if( ! configuredPath )
    {
        CD_ERROR("Path not configured!\n");
        return false;
    }
    if( ! configuredVelocityProfile )
    {
        CD_ERROR("Velocity profile not configured!\n");
        return false;
    }

    currentTrajectory = new KDL::Trajectory_Segment(path, velocityProfile, duration);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlTrajectory::destroy()
{
    delete currentTrajectory; // deletes aggregated path and profile instances, too
    currentTrajectory = 0;
    path = 0;
    orient = 0;
    velocityProfile = 0;

    duration = DURATION_NOT_SET;
    configuredPath = configuredVelocityProfile = false;

    frames.clear();

    return true;
}

// -----------------------------------------------------------------------------

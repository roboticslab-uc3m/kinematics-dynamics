// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include "LogComponent.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool CartesianControlClientROS2::stat(std::vector<double> & x, int * state, double * timestamp)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::inv(const std::vector<double> & xd, std::vector<double> & q)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movj(const std::vector<double> & xd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::relj(const std::vector<double> & xd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movl(const std::vector<double> & xd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movv(const std::vector<double> & xdotd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::gcmp()
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::forc(const std::vector<double> & fd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::stopControl()
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::wait(double timeout)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::tool(const std::vector<double> & x)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::act(int command)
{
    return false;
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::pose(const std::vector<double> & x)
{
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::twist(const std::vector<double> & xdot)
{
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::wrench(const std::vector<double> & w)
{
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::setParameter(int vocab, double value)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::getParameter(int vocab, double * value)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::setParameters(const std::map<int, double> & params)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::getParameters(std::map<int, double> & params)
{
    return false;
}

// -----------------------------------------------------------------------------

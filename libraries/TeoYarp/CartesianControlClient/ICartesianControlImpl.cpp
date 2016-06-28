// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool teo::CartesianControlClient::stat(int &state, std::vector<double> &x)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::CartesianControlClient::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::CartesianControlClient::movj(const std::vector<double> &xd)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::CartesianControlClient::movl(const std::vector<double> &xd)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::CartesianControlClient::stop()
{
    return true;
}

// -----------------------------------------------------------------------------

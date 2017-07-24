// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <vector>

// ------------------- RateThread related ------------------------------------

void roboticslab::CartesianControlServer::run()
{
    std::vector<double> x;
    int state;

    if (!iCartesianControl->stat(state, x))
    {
        return;
    }

    yarp::os::Bottle out;

    for (size_t i = 0; i < x.size(); i++)
    {
        out.addDouble(x[i]);
    }

    fkOutPort.write(out);

    return;
}

// -----------------------------------------------------------------------------

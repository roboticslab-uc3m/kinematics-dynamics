// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <vector>

// ------------------- PeriodicThread related ------------------------------------

void roboticslab::CartesianControlServer::run()
{
    std::vector<double> x;
    int state;
    double timestamp;

    if (!iCartesianControl->stat(x, &state, &timestamp))
    {
        return;
    }

    yarp::os::Bottle &out = fkOutPort.prepare();
    out.clear();
    out.addVocab(state);

    for (size_t i = 0; i < x.size(); i++)
    {
        out.addFloat64(x[i]);
    }

    out.addFloat64(timestamp);

    fkOutPort.write();

    return;
}

// -----------------------------------------------------------------------------

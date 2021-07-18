// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <vector>

#include <yarp/conf/version.h>

using namespace roboticslab;

// ------------------- PeriodicThread related ------------------------------------

void CartesianControlServer::run()
{
    std::vector<double> x;
    int state;
    double timestamp;

    if (!iCartesianControl->stat(x, &state, &timestamp))
    {
        return;
    }

    yarp::os::Bottle & out = fkOutPort.prepare();
    out.clear();
#if YARP_VERSION_MINOR >= 5
    out.addVocab32(state);
#else
    out.addVocab(state);
#endif

    for (size_t i = 0; i < x.size(); i++)
    {
        out.addFloat64(x[i]);
    }

    out.addFloat64(timestamp);

    fkOutPort.write();

    return;
}

// -----------------------------------------------------------------------------

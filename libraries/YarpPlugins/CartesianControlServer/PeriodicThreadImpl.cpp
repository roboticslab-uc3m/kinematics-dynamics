// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

using namespace roboticslab;

// ------------------- PeriodicThread related ------------------------------------

void CartesianControlServer::run()
{
    if (iCartesianControl)
    {
        std::vector<double> x;
        ICartesianControl::State state;
        double timestamp;

        if (!iCartesianControl->getState(x, state, timestamp))
        {
            return;
        }

        yarp::os::Bottle & out = fkOutPort.prepare();
        out.clear();
        out.addVocab32(static_cast<yarp::conf::vocab32_t>(state));

        for (auto i = 0; i < x.size(); i++)
        {
            out.addFloat64(x[i]);
        }

        out.addFloat64(timestamp);
        fkOutPort.write();
    }
}

// -----------------------------------------------------------------------------

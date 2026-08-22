// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

using namespace roboticslab;

// ------------------- PeriodicThread related ------------------------------------

void CartesianControlServer::run()
{
    ICartesianControl::ControllerState state;

    if (iCartesianControl && iCartesianControl->getState(state))
    {
        yarp::os::Bottle & out = fkOutPort.prepare();
        out.clear();
        out.addVocab32(static_cast<yarp::conf::vocab32_t>(state.mode));

        for (auto i = 0; i < state.x.size(); i++)
        {
            out.addFloat64(state.x[i]);
        }

        out.addFloat64(state.timestamp);
        out.addFloat32(state.progress);
        out.addInt8(static_cast<std::int8_t>(state.success));

        fkOutPort.write();
    }
}

// -----------------------------------------------------------------------------

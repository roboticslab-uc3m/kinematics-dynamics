// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CartesianControlServer::attach(yarp::dev::PolyDriver * poly)
{
    if (poly == nullptr)
    {
        yCError(CCS) << "attach() received nullptr";
        return false;
    }

    if (!poly->isValid())
    {
        yCError(CCS) << "attach() received invalid PolyDriver";
        return false;
    }

    if (!poly->view(iCartesianControl))
    {
        yCError(CCS) << "attach() failed to obtain ICartesianControl interface";
        return false;
    }

    rpcResponder->setHandle(iCartesianControl);
    streamResponder->setHandle(iCartesianControl);

    rpcServer.setReader(*rpcResponder);
    commandPort.useCallback(*streamResponder);

    if (rpcTransformResponder)
    {
        rpcTransformResponder->setHandle(iCartesianControl);
        rpcTransformServer.setReader(*rpcTransformResponder);
    }

    if (!fkOutPort.isClosed())
    {
        return yarp::os::PeriodicThread::start();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlServer::detach()
{
    fkOutPort.interrupt();
    yarp::os::PeriodicThread::stop();

    rpcServer.interrupt();
    commandPort.interrupt();

    rpcResponder->unsetHandle();
    streamResponder->unsetHandle();

    if (rpcTransformResponder)
    {
        rpcTransformServer.interrupt();
        rpcTransformResponder->unsetHandle();
    }

    commandPort.interrupt();

    iCartesianControl = nullptr;
    return true;
}

// -----------------------------------------------------------------------------

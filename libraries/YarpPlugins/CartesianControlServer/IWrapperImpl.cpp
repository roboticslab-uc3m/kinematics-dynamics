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

    return configureHandle();
}

// -----------------------------------------------------------------------------

bool CartesianControlServer::configureHandle()
{
    if (!iCartesianControl)
    {
        yCError(CCS) << "Invalid ICartesianControl interface";
        return false;
    }

    rpcResponder->setHandle(iCartesianControl);
    streamResponder->setHandle(iCartesianControl);

    if (rpcTransformResponder)
    {
        rpcTransformResponder->setHandle(iCartesianControl);
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
    yarp::os::PeriodicThread::stop();
    iCartesianControl = nullptr;
    return true;
}

// -----------------------------------------------------------------------------

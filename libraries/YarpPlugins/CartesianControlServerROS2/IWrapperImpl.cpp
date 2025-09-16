// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::attach(yarp::dev::PolyDriver * poly)
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

    if (!poly->view(m_iCartesianControl))
    {
        yCError(CCS) << "attach() failed to obtain ICartesianControl interface";
        return false;
    }

    if (!configureRosHandlers() || !configureRosParameters())
    {
        yCError(CCS) << "Failed to configure ROS handlers and parameters";
        destroyRosHandlers(); // cleanup
        return false;
    }

    return yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::detach()
{
    yarp::os::PeriodicThread::stop();
    destroyRosHandlers();
    m_iCartesianControl = nullptr;
    return true;
}

// -----------------------------------------------------------------------------

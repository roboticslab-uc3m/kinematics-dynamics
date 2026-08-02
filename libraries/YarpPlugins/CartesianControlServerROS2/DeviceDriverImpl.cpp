// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <string>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab::ros2utils;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlServerROS2::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(CCS) << "Failed to parse parameters";
        return false;
    }

    if (m_fkPeriod > 0)
    {
        yarp::os::PeriodicThread::setPeriod(m_fkPeriod * 0.001);
    }
    else
    {
        yCWarning(CCS) << "Invalid period, using default";
        yarp::os::PeriodicThread::setPeriod(std::stoi(m_fkPeriod_defaultValue) * 0.001);
    }

    m_node = createNode(m_name);
    m_spinner = std::make_unique<Spinner>(m_node);

    return m_spinner->start();
}

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::close()
{
    if (m_spinner)
    {
        return m_spinner->stop();
    }

    return true;
}

// -----------------------------------------------------------------------------

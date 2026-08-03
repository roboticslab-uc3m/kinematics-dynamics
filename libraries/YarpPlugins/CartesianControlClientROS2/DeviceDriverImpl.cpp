// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab::ros2utils;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlClientROS2::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(CCC) << "Failed to parse parameters";
        return false;
    }

    m_node = createNode(m_local);
    m_spinner = std::make_unique<Spinner>(m_node);

    return m_spinner->start() && configureRosHandlers();
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::close()
{
    if (m_spinner)
    {
        return m_spinner->stop();
    }

    return true;
}

// -----------------------------------------------------------------------------

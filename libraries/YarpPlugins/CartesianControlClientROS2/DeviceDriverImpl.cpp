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

    if (m_local.empty() || m_remote.empty())
    {
        yCError(CCC) << "Non-empty local and remote names must be specified";
        return false;
    }

    if (m_local[0] == '/')
    {
        m_local = m_local.substr(1);
    }

    if (std::size_t pos = m_local.find('/'); pos != std::string::npos)
    {
        while (pos != std::string::npos)
        {
            m_local.replace(pos, 1, "_");
            pos = m_local.find('/', pos + 1);
        }
    }

    if (m_remote[0] != '/')
    {
        m_remote = '/' + m_remote;
    }

    m_node = createNode(m_local);
    m_spinner = std::make_unique<Spinner>(m_node);

    return m_spinner->start() && configureRosHandlers() && populateRosParameters();
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::close()
{
    m_spinner.reset(); // calls rclcpp::shutdown() in destructor
    return true;
}

// -----------------------------------------------------------------------------

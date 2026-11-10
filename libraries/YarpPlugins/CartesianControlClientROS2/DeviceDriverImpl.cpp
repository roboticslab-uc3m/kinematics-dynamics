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

    if (m_name.empty())
    {
        yCError(CCC) << "Non-empty local node name must be specified";
        return false;
    }

    if (m_name[0] == '/')
    {
        m_name = m_name.substr(1);
    }

    if (auto pos = m_name.find('/'); pos != std::string::npos)
    {
        while (pos != std::string::npos)
        {
            m_name.replace(pos, 1, "_");
            pos = m_name.find('/', pos + 1);
        }
    }

    if (!m_ns.empty() && m_ns[0] != '/')
    {
        m_ns = '/' + m_ns;
    }

    if (m_remote.empty())
    {
        yCError(CCC) << "Non-empty remote node name must be specified";
        return false;
    }

    if (m_remote[0] != '/')
    {
        m_remote = '/' + m_remote;
    }

    m_node = createNode(m_name);
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

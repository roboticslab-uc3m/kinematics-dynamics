// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <string>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

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

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // ROS2 initialization
    m_node = std::make_shared<rclcpp::Node>(m_name);
    m_spinner = new Spinner(m_node);

    return m_spinner->start();
}

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::close()
{
    bool ret = true;

    if (m_spinner)
    {
        ret = m_spinner->stop();
        delete m_spinner;
        m_spinner = nullptr;
    }

    return ret;
}

// -----------------------------------------------------------------------------

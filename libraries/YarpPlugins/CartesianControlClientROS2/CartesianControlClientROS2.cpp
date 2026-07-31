// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::configureRosHandlers()
{
    const auto prefix = "/" + m_remote;

    m_subscription_state = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        prefix + "/state/pose",
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            std::lock_guard lock(m_state_mutex);
            m_last_pose = *msg;
        }
    );

    if (!m_subscription_state)
    {
        yCError(CCC) << "Failed to create subscription for state pose";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

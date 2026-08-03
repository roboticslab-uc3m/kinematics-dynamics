// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::configureRosHandlers()
{
    const auto prefix = "/" + m_remote;

    m_pose = m_node->create_publisher<geometry_msgs::msg::Pose>(prefix + "/command/pose", 10);
    m_twist = m_node->create_publisher<geometry_msgs::msg::Twist>(prefix + "/command/twist", 10);
    m_wrench = m_node->create_publisher<geometry_msgs::msg::Wrench>(prefix + "/command/wrench", 10);
    m_movj = m_node->create_publisher<geometry_msgs::msg::Pose>(prefix + "/command/movj", 10);
    m_relj = m_node->create_publisher<geometry_msgs::msg::Pose>(prefix + "/command/relj", 10);
    m_movl = m_node->create_publisher<geometry_msgs::msg::Pose>(prefix + "/command/movl", 10);
    m_movv = m_node->create_publisher<geometry_msgs::msg::Twist>(prefix + "/command/movv", 10);
    m_forc = m_node->create_publisher<geometry_msgs::msg::Wrench>(prefix + "/command/forc", 10);
    m_tool = m_node->create_publisher<geometry_msgs::msg::Pose>(prefix + "/command/tool", 10);
    m_act = m_node->create_publisher<std_msgs::msg::Int32>(prefix + "/command/gripper", 10);

    m_subscription_state = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        prefix + "/state/pose",
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            std::lock_guard lock(m_mutex_state);
            m_pose_last = *msg;
        }
    );

    if (!m_subscription_state)
    {
        yCError(CCC) << "Failed to create subscription for state pose";
        return false;
    }

    m_client_get_params = m_node->create_client<rcl_interfaces::srv::GetParameters>(prefix + "/get_parameters");

    while (!m_client_get_params->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for the service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Parameter service (get) not available, waiting again...";
    }

    m_client_set_params = m_node->create_client<rcl_interfaces::srv::SetParameters>(prefix + "/set_parameters");

    while (!m_client_set_params->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for the service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Parameter service (set) not available, waiting again...";
    }

    return true;
}

// -----------------------------------------------------------------------------

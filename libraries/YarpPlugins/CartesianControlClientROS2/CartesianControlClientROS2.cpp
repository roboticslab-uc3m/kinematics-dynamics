// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::configureRosHandlers()
{
    m_pose = m_node->create_publisher<geometry_msgs::msg::Pose>(m_remote + "/command/pose", 10);
    m_twist = m_node->create_publisher<geometry_msgs::msg::Twist>(m_remote + "/command/twist", 10);
    m_wrench = m_node->create_publisher<geometry_msgs::msg::Wrench>(m_remote + "/command/wrench", 10);
    m_movj = m_node->create_publisher<geometry_msgs::msg::Pose>(m_remote + "/command/movj", 10);
    m_relj = m_node->create_publisher<geometry_msgs::msg::Pose>(m_remote + "/command/relj", 10);
    m_movl = m_node->create_publisher<geometry_msgs::msg::Pose>(m_remote + "/command/movl", 10);
    m_movv = m_node->create_publisher<geometry_msgs::msg::Twist>(m_remote + "/command/movv", 10);
    m_forc = m_node->create_publisher<geometry_msgs::msg::Wrench>(m_remote + "/command/forc", 10);
    m_tool = m_node->create_publisher<geometry_msgs::msg::Pose>(m_remote + "/command/tool", 10);
    m_act = m_node->create_publisher<std_msgs::msg::Int32>(m_remote + "/command/gripper", 10);

    m_subscription_state = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        m_remote + "/state/pose",
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

    m_client_inv = m_node->create_client<rl_cartesian_control_msgs::srv::Inv>(m_remote + "/inv");

    while (!m_client_inv->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for inverse kinematics service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Inverse kinematics service not available, waiting again...";
    }

    m_client_gcmp = m_node->create_client<std_srvs::srv::Trigger>(m_remote + "/gcmp");

    while (!m_client_gcmp->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for gravity compensation service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Gravity compensation service not available, waiting again...";
    }

    m_client_stop = m_node->create_client<std_srvs::srv::Trigger>(m_remote + "/stop");

    while (!m_client_stop->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for stop service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Stop service not available, waiting again...";
    }

    m_client_get_params = m_node->create_client<rcl_interfaces::srv::GetParameters>(m_remote + "/get_parameters");

    while (!m_client_get_params->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for get parameters service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Parameter service (get) not available, waiting again...";
    }

    m_client_set_params = m_node->create_client<rcl_interfaces::srv::SetParameters>(m_remote + "/set_parameters");

    while (!m_client_set_params->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for set parameters service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Parameter service (set) not available, waiting again...";
    }

    return true;
}

// -----------------------------------------------------------------------------

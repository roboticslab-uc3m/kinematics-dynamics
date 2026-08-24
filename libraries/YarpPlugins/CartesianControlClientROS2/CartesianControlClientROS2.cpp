// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include <yarp/os/LogStream.h>

#include <rcl_interfaces/srv/list_parameters.hpp>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto TIMEOUT = std::chrono::seconds(1);

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::configureRosHandlers()
{
    m_pose = m_node->create_publisher<geometry_msgs::msg::Pose>(m_remote + "/command/pose", 10);
    m_twist = m_node->create_publisher<geometry_msgs::msg::Twist>(m_remote + "/command/twist", 10);
    m_wrench = m_node->create_publisher<geometry_msgs::msg::Wrench>(m_remote + "/command/wrench", 10);

    m_state = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        m_remote + "/state/pose",
        10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
        {
            std::lock_guard lock(m_mutex_state);
            m_pose_last = *msg;
        }
    );

    if (!m_state)
    {
        yCError(CCC) << "Failed to create subscription for state pose";
        return false;
    }

    m_trajectory = rclcpp_action::create_client<rl_cartesian_control_msgs::action::Trajectory>(m_node, m_remote + "/trajectory");

    while (!m_trajectory->wait_for_action_server(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for trajectory action server. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Trajectory action server not available, waiting again...";
    }

    m_move_v = m_node->create_client<rl_cartesian_control_msgs::srv::MoveV>(m_remote + "/move_v");

    while (!m_move_v->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for move_v service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "MoveV service not available, waiting again...";
    }

    m_force = m_node->create_client<rl_cartesian_control_msgs::srv::Force>(m_remote + "/force");

    while (!m_force->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for force service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Force service not available, waiting again...";
    }

    m_tool = m_node->create_client<rl_cartesian_control_msgs::srv::Tool>(m_remote + "/tool");

    while (!m_tool->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for tool service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Tool service not available, waiting again...";
    }

    m_inv = m_node->create_client<rl_cartesian_control_msgs::srv::Inv>(m_remote + "/inv");

    while (!m_inv->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for inverse kinematics service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Inverse kinematics service not available, waiting again...";
    }

    m_act = m_node->create_client<rl_cartesian_control_msgs::srv::Act>(m_remote + "/act");

    while (!m_act->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for actuate tool service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Actuate tool service not available, waiting again...";
    }

    m_gcmp = m_node->create_client<std_srvs::srv::Trigger>(m_remote + "/gcmp");

    while (!m_gcmp->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for gravity compensation service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Gravity compensation service not available, waiting again...";
    }

    m_stop = m_node->create_client<std_srvs::srv::Trigger>(m_remote + "/stop");

    while (!m_stop->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for stop service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Stop service not available, waiting again...";
    }

    m_get_params = m_node->create_client<rcl_interfaces::srv::GetParameters>(m_remote + "/get_parameters");

    while (!m_get_params->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for get parameters service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Parameter service (get) not available, waiting again...";
    }

    m_set_params = m_node->create_client<rcl_interfaces::srv::SetParameters>(m_remote + "/set_parameters");

    while (!m_set_params->wait_for_service(TIMEOUT))
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

bool CartesianControlClientROS2::populateRosParameters()
{
    auto client = m_node->create_client<rcl_interfaces::srv::ListParameters>(m_remote + "/list_parameters");

    while (!client->wait_for_service(TIMEOUT))
    {
        if (!rclcpp::ok())
        {
            yCError(CCC) << "Interrupted while waiting for list parameters service. Exiting.";
            return false;
        }

        yCInfo(CCC) << "Parameter service (list) not available, waiting again...";
    }

    auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    auto result = client->async_send_request(request);
    auto response = result.get();

    m_supported_parameters = response->result.names;

    return true;
}

// -----------------------------------------------------------------------------

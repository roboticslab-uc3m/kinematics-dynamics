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
    m_state = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        m_remote + "/state/pose",
        10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
        {
            std::lock_guard lock(m_mutex_state);
            m_pose_last = *msg;
        }
    );

    m_pose = m_node->create_publisher<geometry_msgs::msg::Pose>(m_remote + "/command/pose", 10);
    m_twist = m_node->create_publisher<geometry_msgs::msg::Twist>(m_remote + "/command/twist", 10);
    m_wrench = m_node->create_publisher<geometry_msgs::msg::Wrench>(m_remote + "/command/wrench", 10);

    m_move_v = m_node->create_client<rl_cartesian_control_msgs::srv::MoveV>(m_remote + "/move_velocity");
    m_force = m_node->create_client<rl_cartesian_control_msgs::srv::Force>(m_remote + "/force_control");
    m_tool = m_node->create_client<rl_cartesian_control_msgs::srv::Tool>(m_remote + "/change_tool");
    m_inv = m_node->create_client<rl_cartesian_control_msgs::srv::Inv>(m_remote + "/solve_pose");
    m_act = m_node->create_client<rl_cartesian_control_msgs::srv::Act>(m_remote + "/actuate_tool");
    m_gcmp = m_node->create_client<std_srvs::srv::Trigger>(m_remote + "/gravity_compensation");
    m_stop = m_node->create_client<std_srvs::srv::Trigger>(m_remote + "/stop_control");

    m_get_params = m_node->create_client<rcl_interfaces::srv::GetParameters>(m_remote + "/get_parameters");
    m_set_params = m_node->create_client<rcl_interfaces::srv::SetParameters>(m_remote + "/set_parameters");

    m_trajectory = rclcpp_action::create_client<rl_cartesian_control_msgs::action::PoseTrajectory>(m_node, m_remote + "/trajectory/pose");

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

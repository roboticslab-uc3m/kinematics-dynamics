// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_ROS2_HPP__
#define __CARTESIAN_CONTROL_SERVER_ROS2_HPP__

#include <string>

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <kdl/frames.hpp>

#include <rl_cartesian_control_msgs/srv/actuate_tool.hpp>
#include <rl_cartesian_control_msgs/srv/change_tool.hpp>
#include <rl_cartesian_control_msgs/srv/force_control.hpp>
#include <rl_cartesian_control_msgs/srv/move_velocity.hpp>
#include <rl_cartesian_control_msgs/srv/solve_pose.hpp>

#include <rl_cartesian_control_msgs/action/pose_trajectory.hpp>

#include "Ros2Utils.hpp"
#include "ICartesianControl.h"
#include "CartesianControlServerROS2_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CartesianControlServerROS2
 *
 * @brief Contains CartesianControlServerROS2.
 */

class CartesianControlServerROS2 : public yarp::dev::DeviceDriver,
                                   public yarp::dev::WrapperSingle,
                                   public yarp::os::PeriodicThread,
                                   public CartesianControlServerROS2_ParamsParser
{
public:
    CartesianControlServerROS2() : yarp::os::PeriodicThread(1.0)
    {}

    // Implementation in DeviceDriverImpl.cpp
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // Implementation in IWrapperImpl.cpp
    bool attach(yarp::dev::PolyDriver * poly) override;
    bool detach() override;

    // Implementation in PeriodicThread.cpp
    void run() override;

private:
    bool configureRosHandlers();
    bool configureRosParameters();
    void destroyRosHandlers();

    roboticslab::ICartesianControl * m_iCartesianControl;

    roboticslab::ros2utils::Spinner::Ptr m_spinner;

    rclcpp::Node::SharedPtr m_node;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_state;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_pose;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twist;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr m_wrench;

    rclcpp::Service<rl_cartesian_control_msgs::srv::MoveVelocity>::SharedPtr m_move_v;
    rclcpp::Service<rl_cartesian_control_msgs::srv::ForceControl>::SharedPtr m_force;
    rclcpp::Service<rl_cartesian_control_msgs::srv::ChangeTool>::SharedPtr m_tool;
    rclcpp::Service<rl_cartesian_control_msgs::srv::SolvePose>::SharedPtr m_inv;
    rclcpp::Service<rl_cartesian_control_msgs::srv::ActuateTool>::SharedPtr m_act;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_gcmp;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_stop;

    rclcpp_action::Server<rl_cartesian_control_msgs::action::PoseTrajectory>::SharedPtr m_trajectory;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rl_cartesian_control_msgs::action::PoseTrajectory>> m_goalHandle;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_params;

    rcl_interfaces::msg::SetParametersResult params_cb(const std::vector<rclcpp::Parameter> & parameters);
};

#endif // __CARTESIAN_CONTROL_SERVER_ROS2_HPP__

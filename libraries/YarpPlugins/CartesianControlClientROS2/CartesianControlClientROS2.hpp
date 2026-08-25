// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__
#define __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <yarp/dev/Drivers.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include <rl_cartesian_control_msgs/srv/move_v.hpp>
#include <rl_cartesian_control_msgs/srv/force.hpp>
#include <rl_cartesian_control_msgs/srv/tool.hpp>
#include <rl_cartesian_control_msgs/srv/inv.hpp>
#include <rl_cartesian_control_msgs/srv/act.hpp>

#include <rl_cartesian_control_msgs/action/trajectory.hpp>

#include "Ros2Utils.hpp"
#include "ICartesianControl.h"
#include "CartesianControlClientROS2_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CartesianControlClientROS2
 *
 * @brief Contains CartesianControlClientROS2.
 */

/**
 * @ingroup CartesianControlClientROS2
 * @brief The CartesianControlClientROS2 class implements ICartesianControl client side.
 */
class CartesianControlClientROS2 : public yarp::dev::DeviceDriver,
                                   public roboticslab::ICartesianControl,
                                   public CartesianControlClientROS2_ParamsParser
{
public:
    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --

    // RPC commands
    yarp::dev::ReturnValue getState(roboticslab::ICartesianControl::ControllerState & state) override;
    yarp::dev::ReturnValue solvePose(const std::vector<double> & xd, std::vector<double> & q) override;
    yarp::dev::ReturnValue moveJoint(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveLinear(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveVelocity(const std::vector<double> & xdotd) override;
    yarp::dev::ReturnValue gravityCompensation() override;
    yarp::dev::ReturnValue forceControl(const std::vector<double> & fd) override;
    yarp::dev::ReturnValue stopControl() override;
    yarp::dev::ReturnValue changeTool(const std::vector<double> & x) override;
    yarp::dev::ReturnValue actuateTool(roboticslab::ICartesianControl::Actuator command) override;

    // streaming commands
    void pose(const std::vector<double> & x) override;
    void twist(const std::vector<double> & xdot) override;
    void wrench(const std::vector<double> & w) override;

    // configuration getters/setters
    yarp::dev::ReturnValue setParameter(roboticslab::ICartesianControl::Config vocab, double value) override;
    yarp::dev::ReturnValue getParameter(roboticslab::ICartesianControl::Config vocab, double * value) override;
    yarp::dev::ReturnValue setParameters(const std::map<roboticslab::ICartesianControl::Config, double> & params) override;
    yarp::dev::ReturnValue getParameters(std::map<roboticslab::ICartesianControl::Config, double> & params) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    bool configureRosHandlers();
    bool populateRosParameters();
    yarp::dev::ReturnValue sendTrajectoryGoal(roboticslab::ICartesianControl::Mode mode, const std::vector<double> & xd);

    rclcpp::Node::SharedPtr m_node;
    roboticslab::ros2utils::Spinner::Ptr m_spinner;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_pose;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr m_wrench;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_state;

    rclcpp_action::Client<rl_cartesian_control_msgs::action::Trajectory>::SharedPtr m_trajectory;

    rclcpp::Client<rl_cartesian_control_msgs::srv::MoveV>::SharedPtr m_move_v;
    rclcpp::Client<rl_cartesian_control_msgs::srv::Force>::SharedPtr m_force;
    rclcpp::Client<rl_cartesian_control_msgs::srv::Tool>::SharedPtr m_tool;
    rclcpp::Client<rl_cartesian_control_msgs::srv::Inv>::SharedPtr m_inv;
    rclcpp::Client<rl_cartesian_control_msgs::srv::Act>::SharedPtr m_act;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_gcmp;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_stop;

    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr m_get_params;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr m_set_params;

    std::mutex m_mutex_state;
    geometry_msgs::msg::PoseStamped m_pose_last;
    std::atomic<float> m_progress {1.0f};
    std::atomic<bool> m_success {false};

    std::vector<std::string> m_supported_parameters;
};

#endif // __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__

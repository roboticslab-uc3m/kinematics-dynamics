// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__
#define __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__

#include <mutex>
#include <string>
#include <vector>

#include <yarp/dev/Drivers.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include <rl_cartesian_control_msgs/srv/inv.hpp>

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
    yarp::dev::ReturnValue stat(std::vector<double> & x, roboticslab::ICartesianControl::State * state = nullptr, double * timestamp = nullptr) override;
    yarp::dev::ReturnValue inv(const std::vector<double> & xd, std::vector<double> & q) override;
    yarp::dev::ReturnValue movj(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue relj(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue movl(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue movv(const std::vector<double> & xdotd) override;
    yarp::dev::ReturnValue gcmp() override;
    yarp::dev::ReturnValue forc(const std::vector<double> & fd) override;
    yarp::dev::ReturnValue stopControl() override;
    yarp::dev::ReturnValue wait(double timeout) override;
    yarp::dev::ReturnValue tool(const std::vector<double> & x) override;
    yarp::dev::ReturnValue act(roboticslab::ICartesianControl::Actuator command) override;

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

    rclcpp::Node::SharedPtr m_node;
    roboticslab::ros2utils::Spinner::Ptr m_spinner;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_pose;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr m_wrench;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_movj;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_relj;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_movl;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_movv;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr m_forc;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_tool;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_act;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_subscription_state;

    rclcpp::Client<rl_cartesian_control_msgs::srv::Inv>::SharedPtr m_client_inv;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_client_gcmp;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_client_stop;

    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr m_client_get_params;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr m_client_set_params;

    std::mutex m_mutex_state;
    geometry_msgs::msg::PoseStamped m_pose_last;

    std::vector<std::string> m_supported_parameters;

    enum gripper_state { GRIPPER_NONE, GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_STOP };
};

#endif // __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__

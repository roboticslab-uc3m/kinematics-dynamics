// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_ROS2_HPP__
#define __CARTESIAN_CONTROL_SERVER_ROS2_HPP__

#include <string>

#include <yarp/os/Thread.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <kdl/frames.hpp>

#include "Spinner.hpp"
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
    void destroyRosHandlers();

    roboticslab::ICartesianControl * m_iCartesianControl;

    Spinner * m_spinner;

    rclcpp::Node::SharedPtr m_node;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_stat;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_pose;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twist;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr m_wrench;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_movj;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_movl;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_movv;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_forc;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_tool;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_act;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_gcmp;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_stop;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_params;
    int preset_streaming_cmd;
    roboticslab::ICartesianSolver::reference_frame frame;

    void pose_cb(const geometry_msgs::msg::Pose::SharedPtr msg);
    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wrench_cb(const geometry_msgs::msg::Wrench::SharedPtr msg);

    void movj_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void movl_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void movv_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void forc_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void tool_cb(const geometry_msgs::msg::Pose::SharedPtr msg);
    void act_cb(const std_msgs::msg::Int32::SharedPtr msg);

    void gcmp_cb(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
    void stop_cb(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

    rcl_interfaces::msg::SetParametersResult params_cb(const std::vector<rclcpp::Parameter> &parameters);

    // Note that the order of gripper_state enum values must match the order from spacenav_device. If modifying this, please update.
    enum gripper_state { GRIPPER_NONE, GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_STOP };
};

#endif // __CARTESIAN_CONTROL_SERVER_ROS2_HPP__

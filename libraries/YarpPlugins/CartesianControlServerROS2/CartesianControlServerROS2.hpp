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

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_posePublisher;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_poseSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twistSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr m_wrenchSubscription;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_movjSubscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_movlSubscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_movvSubscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_forcSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_toolSubscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_actSubscription;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_gcmpService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_stopService;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameterServer;
    std::string preset_streaming_cmd;
    std::string frame;

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wrench_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    void movj_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void movl_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void movv_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void forc_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void tool_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void act_callback(const std_msgs::msg::Int32::SharedPtr msg);

    void gcmp_callback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
    void stop_callback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);

    // Note that the order of gripper_state enum values must match the order from spacenav_device. If modifying this, please update.
    enum gripper_state { GRIPPER_NONE, GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_STOP };
};

#endif // __CARTESIAN_CONTROL_SERVER_ROS2_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_ROS2_HPP__
#define __CARTESIAN_CONTROL_SERVER_ROS2_HPP__

#include <string>

#include <yarp/os/Thread.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <kdl/frames.hpp>

#include "ICartesianControl.h"
#include "CartesianControlServerROS2_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CartesianControlServerROS2
 *
 * @brief Contains CartesianControlServerROS2.
 */

class Spinner : public yarp::os::Thread
{
public:
    Spinner(std::shared_ptr<rclcpp::Node> input_node);
    ~Spinner() override;
    void run() override;

private:
    bool m_spun {false};
    std::shared_ptr<rclcpp::Node> m_node;
};

// -----------------------------------------------------------------------------

class CartesianControlServerROS2 : public yarp::dev::DeviceDriver,
                                   public yarp::os::PeriodicThread,
                                   public CartesianControlServerROS2_ParamsParser
{
public:
    CartesianControlServerROS2() : yarp::os::PeriodicThread(1.0)
    {}

    // Implementation in DeviceDriverImpl.cpp
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // Implementation in PeriodicThread.cpp
    void run() override;

private:
    // Devices
    yarp::dev::PolyDriver cartesianControlDevice;
    roboticslab::ICartesianControl * m_iCartesianControl;

    // ROS2 attributes
    Spinner * m_spinner;

    rclcpp::Node::SharedPtr                                       m_node;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr     m_poseSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    m_twistSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr   m_wrenchSubscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr         m_gripperSubscription;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   m_movj;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   m_movl;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::string preset_streaming_cmd;
    std::string frame;

    // Subscription callbacks - Topics
    void poseTopic_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void twistTopic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wrenchTopic_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void gripperTopic_callback(const std_msgs::msg::Int32::SharedPtr msg);

    void movj_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void movl_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);

    /*Notice that order of gripper_state enum values matches the same order from spacenav_device. If modify, please, update*/
    enum gripper_state { GRIPPER_NONE, GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_STOP };
};

#endif // __CARTESIAN_CONTROL_SERVER_ROS2_HPP__

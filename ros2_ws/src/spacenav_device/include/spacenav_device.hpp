// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <vector>
#include <string>
#include <mutex>

#include <ICartesianControl.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <std_msgs/msg/int32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using SetParameters = rcl_interfaces::srv::SetParameters;
using Parameter = rcl_interfaces::msg::Parameter;
using ParameterValue = rcl_interfaces::msg::ParameterValue;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

class SpacenavSubscriber : public rclcpp::Node
{
public:
    SpacenavSubscriber();
    ~SpacenavSubscriber();

private:
    void spnav_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool set_preset_streaming_cmd(const std::string &value); 
    SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);
    void timer_callback();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_spnav_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_state_pose_{nullptr};
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_spnav_twist_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_spnav_pose_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_spnav_wrench_{nullptr};
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_spnav_gripper_{nullptr};

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_{nullptr};
    rclcpp::Client<SetParameters>::SharedPtr client_param_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    geometry_msgs::msg::Twist::SharedPtr last_msg_{nullptr};
    geometry_msgs::msg::Pose::SharedPtr last_msg_pose_{nullptr};
    geometry_msgs::msg::Wrench::SharedPtr last_msg_wrench_{nullptr};

    std::mutex msg_mutex_;

    rclcpp::Time last_update_time_{0};

    tf2::Vector3 initial_position_{0, 0, 0};
    tf2::Vector3 current_position_{0, 0, 0};
    tf2::Quaternion initial_orientation_{0, 0, 0, 1};
    tf2::Quaternion current_orientation_{0, 0, 0, 1};

    bool initial_pose_set_{false};
    bool virtual_pose_set{false};
    bool position_changed_{false};

    roboticslab::ICartesianControl * iCartesianControl_{nullptr};

    double scale_{0.3};
    std::string streaming_msg_{"twist"};

    /*Notice that order of gripper_state enum values matches the same order from CartesianControlServerROS2. If modify, please, update*/
    enum gripper_state { GRIPPER_NONE, GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_STOP };
    gripper_state gripper_state_{GRIPPER_NONE};

};


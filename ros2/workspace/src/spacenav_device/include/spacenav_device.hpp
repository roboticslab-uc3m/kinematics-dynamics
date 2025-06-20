// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <mutex>
#include <string>
#include <vector>

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

/**
 * @ingroup kinematics-dynamics-programs
 * @defgroup SpacenavSubscriber SpacenavSubscriber
 * @brief Creates an instance of SpacenavSubscriber.
 */
class SpacenavSubscriber : public rclcpp::Node
{
public:
    SpacenavSubscriber();

private:
    void spnav_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool set_preset_streaming_cmd(const std::string &value);
    SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);
    void timer_callback();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_spnav_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_state_pose_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_spnav_twist_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_spnav_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_spnav_wrench_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_spnav_gripper_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rclcpp::Client<SetParameters>::SharedPtr client_param_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist::SharedPtr last_msg_;
    geometry_msgs::msg::Pose::SharedPtr last_msg_pose_;
    geometry_msgs::msg::Wrench::SharedPtr last_msg_wrench_;

    std::mutex msg_mutex_;

    rclcpp::Time last_update_time_ {0};

    tf2::Vector3 initial_position_ {0, 0, 0};
    tf2::Vector3 current_position_ {0, 0, 0};

    tf2::Quaternion initial_orientation_ {0, 0, 0, 1};
    tf2::Quaternion current_orientation_ {0, 0, 0, 1};

    bool initial_pose_set_ {false};
    bool virtual_pose_set {false};
    bool position_changed_ {false};

    double scale_ {0.0};
    std::string streaming_msg_;

    /* Notice that order of gripper_state enum values matches the same order from CartesianControlServerROS2. If ever modified, please, update this. */
    enum gripper_state { GRIPPER_NONE, GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_STOP };
    gripper_state gripper_state_ {GRIPPER_NONE};
};

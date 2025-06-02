// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <vector>

#include <kdl/frames.hpp>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- Subscription callbacks ------------------------------------

bool CartesianControlServerROS2::configureSubscriptions()
{
    using namespace std::placeholders;

    auto prefix = "/" + m_name;

    // Parameters
    m_parameterServer = m_node->add_on_set_parameters_callback(std::bind(&CartesianControlServerROS2::parameter_callback, this, _1));

    // Publisher
    m_posePublisher = m_node->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/state/pose", 10);

    // Subscriptions
    m_poseSubscription = m_node->create_subscription<geometry_msgs::msg::Pose>(prefix + "/command/pose", 10,
                                                                               std::bind(&CartesianControlServerROS2::poseTopic_callback,
                                                                               this, _1));
    if (!m_poseSubscription)
    {
        yCError(CCS) << "Could not initialize the pose command subscription";
        return false;
    }

    m_twistSubscription = m_node->create_subscription<geometry_msgs::msg::Twist>(prefix + "/command/twist", 10,
                                                                                 std::bind(&CartesianControlServerROS2::twistTopic_callback,
                                                                                 this, _1));
    if (!m_twistSubscription)
    {
        yCError(CCS) << "Could not initialize the twist command subscription";
        return false;
    }

    m_wrenchSubscription = m_node->create_subscription<geometry_msgs::msg::Wrench>(prefix + "/command/wrench", 10,
                                                                                   std::bind(&CartesianControlServerROS2::wrenchTopic_callback,
                                                                                   this, _1));
    if (!m_wrenchSubscription)
    {
        yCError(CCS) << "Could not initialize the wrench command subscription";
        return false;
    }

    m_gripperSubscription = m_node->create_subscription<std_msgs::msg::Int32>(prefix + "/command/gripper", 10,
                                                                              std::bind(&CartesianControlServerROS2::gripperTopic_callback,
                                                                              this, _1));

    if (!m_gripperSubscription)
    {
        yCError(CCS) << "Could not initialize the gripper command subscription";
        return false;
    }

    m_movjSubscription = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(prefix + "/command/movj", 10,
                                                                                       std::bind(&CartesianControlServerROS2::movj_callback,
                                                                                       this, _1));

    if (!m_movjSubscription)
    {
        yCError(CCS) << "Could not initialize the movj command subscription";
        return false;
    }

    m_movlSubscription = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(prefix + "/command/movl", 10,
                                                                                       std::bind(&CartesianControlServerROS2::movl_callback,
                                                                                       this, _1));

    if (!m_movlSubscription)
    {
        yCError(CCS) << "Could not initialize the movl command subscription";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::cancelSubscriptions()
{
    m_parameterServer.reset();

    m_posePublisher.reset();

    m_poseSubscription.reset();
    m_twistSubscription.reset();
    m_wrenchSubscription.reset();
    m_gripperSubscription.reset();
    m_movjSubscription.reset();
    m_movlSubscription.reset();
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::poseTopic_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    const auto ori = KDL::Rotation::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    const auto rot = ori.GetRot();

    std::vector<double> v {
        msg->position.x,
        msg->position.y,
        msg->position.z,
        rot.x(),
        rot.y(),
        rot.z()
    };

    if (preset_streaming_cmd == "pose")
    {
        yCInfo(CCS) << "Received pose:" << v;
        m_iCartesianControl->pose(v);
    }
    else
    {
        yCWarning(CCS) << "Streaming command not set to 'pose'.";
    }
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::twistTopic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::vector<double> v {
        msg->linear.x,
        msg->linear.y,
        msg->linear.z,
        msg->angular.x,
        msg->angular.y,
        msg->angular.z
    };

    bool zero_msg = v == std::vector<double>(6, 0.0);

    if (preset_streaming_cmd == "twist")
    {
        m_iCartesianControl->twist(v);

        if (!zero_msg)
        {
            yCInfo(CCS) << "Received twist:" << v;
        }
    }
    else
    {
        yCWarning(CCS) << "Streaming command not set to 'twist'.";
    }
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::wrenchTopic_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    std::vector<double> v {
        msg->force.x,
        msg->force.y,
        msg->force.z,
        msg->torque.x,
        msg->torque.y,
        msg->torque.z
    };

    if (preset_streaming_cmd == "wrench")
    {
        yCInfo(CCS) << "Received wrench:" << v;
        m_iCartesianControl->wrench(v);
    }
    else
    {
        yCWarning(CCS) << "Streaming command not set to 'wrench'.";
    }
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::gripperTopic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    switch (msg->data)
    {
    case GRIPPER_CLOSE:
        yCInfo(CCS) << "Gripper close";
        m_iCartesianControl->act(VOCAB_CC_ACTUATOR_CLOSE_GRIPPER);
        break;
    case GRIPPER_OPEN:
        yCInfo(CCS) << "Gripper open";
        m_iCartesianControl->act(VOCAB_CC_ACTUATOR_OPEN_GRIPPER);
        break;
    case GRIPPER_STOP:
        yCInfo(CCS) << "Gripper stop";
        m_iCartesianControl->act(VOCAB_CC_ACTUATOR_STOP_GRIPPER);
        break;
    }
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::movj_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid movj command. Expected 6 elements.";
        return;
    }

    if (preset_streaming_cmd == "none")
    {
        yCInfo(CCS) << "Received movj msg:" << msg->data;
        m_iCartesianControl->movj(msg->data);
    }
    else
    {
        yCWarning(CCS) << "Streaming command not set to 'none'.";
    }
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::movl_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid movl command. Expected 6 elements.";
        return;
    }

    if (preset_streaming_cmd == "none")
    {
        yCInfo(CCS) << "Received movl msg:" << msg->data;
        m_iCartesianControl->movl(msg->data);
    }
    else
    {
        yCWarning(CCS) << "Streaming command not set to 'none'.";
    }
}

// -----------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult CartesianControlServerROS2::parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
    {
        if (param.get_name() == "preset_streaming_cmd")
        {
            preset_streaming_cmd = param.value_to_string();

            if (preset_streaming_cmd == "twist")
            {
                yCInfo(CCS) << "Param for preset_streaming_cmd correctly set:" << preset_streaming_cmd;

                if (!m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_TWIST))
                {
                    yCWarning(CCS) << "Unable to preset streaming command";
                }
            }
            else if (preset_streaming_cmd == "pose")
            {
                yCInfo(CCS) << "Param for preset_streaming_cmd correctly set:" << preset_streaming_cmd;

                if (!m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_POSE))
                {
                    yCWarning(CCS) << "Unable to preset streaming command";
                }
            }
            else if (preset_streaming_cmd == "wrench")
            {
                yCInfo(CCS) << "Param for preset_streaming_cmd correctly set:" << preset_streaming_cmd;

                if (!m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_WRENCH))
                {
                    yCWarning(CCS) << "Unable to preset streaming command";
                }
            }
            else if (preset_streaming_cmd == "none")
            {
                yCInfo(CCS) << "Param for preset_streaming_cmd correctly set:" << preset_streaming_cmd;

                if (!m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_NOT_SET))
                {
                    yCWarning(CCS) << "Unable to preset streaming command";
                }
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid parameter value. Only 'twist', 'pose', 'wrench' or 'none' are allowed.";
                yCInfo(CCS) << "Invalid parameter value for preset_streaming_cmd.";
            }
        }

        if (param.get_name() == "frame")
        {
            frame = param.value_to_string();

            if (frame == "base")
            {
                yCInfo(CCS) << "Param for frame correctly set:" << frame;

                if (!m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::BASE_FRAME))
                {
                    yCWarning(CCS) << "Unable to preset frame";
                }
            }
            else if (frame == "tcp")
            {
                yCInfo(CCS) << "Param for frame correctly set:" << frame;

                if (!m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::TCP_FRAME))
                {
                    yCWarning(CCS) << "Unable to preset frame";
                }
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid parameter value. Only 'base' or 'tcp' are allowed. Using 'base' as default.";
                yCInfo(CCS) << "Invalid parameter value for frame.";
            }
        }
    }

    return result;
}

// -----------------------------------------------------------------------------

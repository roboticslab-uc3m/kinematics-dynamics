// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <vector>

#include <kdl/frames.hpp>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- Subscription callbacks ------------------------------------

bool CartesianControlServerROS2::configureRosHandlers()
{
    using namespace std::placeholders;
    using ccs = CartesianControlServerROS2;

    const auto prefix = "/" + m_name;

    m_params = m_node->add_on_set_parameters_callback(std::bind(&ccs::params_cb, this, _1));

    m_stat = m_node->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/state/pose", 10);

    m_pose = m_node->create_subscription<geometry_msgs::msg::Pose>(prefix + "/command/pose", 10, std::bind(&ccs::pose_cb, this, _1));

    if (!m_pose)
    {
        yCError(CCS) << "Could not initialize the pose command subscription";
        return false;
    }

    m_twist = m_node->create_subscription<geometry_msgs::msg::Twist>(prefix + "/command/twist", 10, std::bind(&ccs::twist_cb, this, _1));

    if (!m_twist)
    {
        yCError(CCS) << "Could not initialize the twist command subscription";
        return false;
    }

    m_wrench = m_node->create_subscription<geometry_msgs::msg::Wrench>(prefix + "/command/wrench", 10, std::bind(&ccs::wrench_cb, this, _1));

    if (!m_wrench)
    {
        yCError(CCS) << "Could not initialize the wrench command subscription";
        return false;
    }

    m_movj = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(prefix + "/command/movj", 10, std::bind(&ccs::movj_cb, this, _1));

    if (!m_movj)
    {
        yCError(CCS) << "Could not initialize the movj command subscription";
        return false;
    }

    m_movl = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(prefix + "/command/movl", 10, std::bind(&ccs::movl_cb, this, _1));

    if (!m_movl)
    {
        yCError(CCS) << "Could not initialize the movl command subscription";
        return false;
    }

    m_movv = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(prefix + "/command/movv", 10, std::bind(&ccs::movv_cb, this, _1));

    if (!m_movv)
    {
        yCError(CCS) << "Could not initialize the movv command subscription";
        return false;
    }

    m_forc = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(prefix + "/command/forc", 10, std::bind(&ccs::forc_cb, this, _1));

    if (!m_forc)
    {
        yCError(CCS) << "Could not initialize the forc command subscription";
        return false;
    }

    m_tool = m_node->create_subscription<geometry_msgs::msg::Pose>(prefix + "/command/pose", 10, std::bind(&ccs::tool_cb, this, _1));

    if (!m_tool)
    {
        yCError(CCS) << "Could not initialize the tool command subscription";
        return false;
    }

    m_act = m_node->create_subscription<std_msgs::msg::Int32>(prefix + "/command/gripper", 10, std::bind(&ccs::act_cb, this, _1));

    if (!m_act)
    {
        yCError(CCS) << "Could not initialize the gripper command subscription";
        return false;
    }

    m_inv = m_node->create_service<rl_cartesian_control_msgs::srv::Inv>(prefix + "/inv", std::bind(&ccs::inv_cb, this, _1, _2));

    if (!m_inv)
    {
        yCError(CCS) << "Could not initialize the inv service";
        return false;
    }

    m_gcmp = m_node->create_service<std_srvs::srv::Trigger>(prefix + "/gcmp", std::bind(&ccs::gcmp_cb, this, _1, _2));

    if (!m_gcmp)
    {
        yCError(CCS) << "Could not initialize the gcmp service";
        return false;
    }

    m_stop = m_node->create_service<std_srvs::srv::Trigger>(prefix + "/stop", std::bind(&ccs::stop_cb, this, _1, _2));

    if (!m_stop)
    {
        yCError(CCS) << "Could not initialize the stop service";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::destroyRosHandlers()
{
    m_params.reset();

    m_stat.reset();

    m_movj.reset();
    m_movl.reset();
    m_movv.reset();
    m_forc.reset();
    m_tool.reset();
    m_act.reset();

    m_pose.reset();
    m_twist.reset();
    m_wrench.reset();

    m_inv.reset();
    m_gcmp.reset();
    m_stop.reset();
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::pose_cb(const geometry_msgs::msg::Pose::SharedPtr msg)
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

    if (preset_streaming_cmd == VOCAB_CC_POSE)
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

void CartesianControlServerROS2::twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
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

    if (preset_streaming_cmd == VOCAB_CC_TWIST)
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

void CartesianControlServerROS2::wrench_cb(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    std::vector<double> v {
        msg->force.x,
        msg->force.y,
        msg->force.z,
        msg->torque.x,
        msg->torque.y,
        msg->torque.z
    };

    if (preset_streaming_cmd == VOCAB_CC_WRENCH)
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

void CartesianControlServerROS2::movj_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid movj command. Expected 6 elements.";
        return;
    }

    if (preset_streaming_cmd == VOCAB_CC_NOT_SET)
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

void CartesianControlServerROS2::movl_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid movl command. Expected 6 elements.";
        return;
    }

    if (preset_streaming_cmd == VOCAB_CC_NOT_SET)
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

void CartesianControlServerROS2::movv_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid movv command. Expected 6 elements.";
        return;
    }

    if (preset_streaming_cmd == VOCAB_CC_NOT_SET)
    {
        yCInfo(CCS) << "Received movv msg:" << msg->data;
        m_iCartesianControl->movv(msg->data);
    }
    else
    {
        yCWarning(CCS) << "Streaming command not set to 'none'.";
    }
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::forc_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid forc command. Expected 6 elements.";
        return;
    }

    if (preset_streaming_cmd == VOCAB_CC_NOT_SET)
    {
        yCInfo(CCS) << "Received forc msg:" << msg->data;
        m_iCartesianControl->forc(msg->data);
    }
    else
    {
        yCWarning(CCS) << "Streaming command not set to 'none'.";
    }
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::tool_cb(const geometry_msgs::msg::Pose::SharedPtr msg)
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

    m_iCartesianControl->tool(v);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::act_cb(const std_msgs::msg::Int32::SharedPtr msg)
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

void CartesianControlServerROS2::inv_cb(const rl_cartesian_control_msgs::srv::Inv::Request::SharedPtr request, rl_cartesian_control_msgs::srv::Inv::Response::SharedPtr response)
{
    const auto & pose = request->x;
    const auto ori = KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    const auto rot = ori.GetRot();

    std::vector<double> v {
        pose.position.x,
        pose.position.y,
        pose.position.z,
        rot.x(),
        rot.y(),
        rot.z()
    };

    std::vector<double> q;
    response->success = m_iCartesianControl->inv(v, q);
    response->q.data = q;
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::gcmp_cb(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
{
    response->success = m_iCartesianControl->gcmp();
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::stop_cb(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
{
    response->success = m_iCartesianControl->stopControl();
}

// -----------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult CartesianControlServerROS2::params_cb(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
    {
        const auto name = param.get_name();
        const auto value = param.value_to_string();

        if (name == "preset_streaming_cmd")
        {
            if (value != "pose" && value != "twist" && value != "wrench" && value != "none")
            {
                result.successful = false;
                result.reason = "Invalid parameter value. Only 'twist', 'pose', 'wrench' or 'none' are allowed.";
                yCInfo(CCS) << "Invalid parameter value for preset_streaming_cmd.";
                continue;
            }

            int vocab;

            if (value == "twist")
            {
                vocab = VOCAB_CC_TWIST;
            }
            else if (value == "pose")
            {
                vocab = VOCAB_CC_POSE;
            }
            else if (value == "wrench")
            {
                vocab = VOCAB_CC_WRENCH;
            }
            else if (value == "none")
            {
                vocab = VOCAB_CC_NOT_SET;
            }

            if (m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, vocab))
            {
                yCInfo(CCS) << "Param for preset_streaming_cmd correctly set:" << value;
                preset_streaming_cmd = vocab;
            }
            else
            {
                yCWarning(CCS) << "Unable to preset streaming command";
            }
        }
        else if (name == "frame")
        {
            if (value != "base" && value != "tcp")
            {
                result.successful = false;
                result.reason = "Invalid parameter value. Only 'base' or 'tcp' are allowed.";
                yCInfo(CCS) << "Invalid parameter value for frame.";
                continue;
            }

            ICartesianSolver::reference_frame refFrame;

            if (value == "base")
            {
                refFrame = ICartesianSolver::BASE_FRAME;
            }
            else if (value == "tcp")
            {
                refFrame = ICartesianSolver::TCP_FRAME;
            }

            if (m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, refFrame))
            {
                yCInfo(CCS) << "Param for frame correctly set:" << value;
                frame = refFrame;
            }
            else
            {
                yCWarning(CCS) << "Unable to preset frame";
            }
        }
    }

    return result;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <algorithm> // std::transform
#include <vector>

#include <kdl/frames.hpp>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::configureRosHandlers()
{
    using namespace std::placeholders;
    using ccs = CartesianControlServerROS2;

    const auto prefix = "/" + m_name;

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

bool CartesianControlServerROS2::configureRosParameters()
{
    std::map<int, double> params;

    if (!m_iCartesianControl->getParameters(params))
    {
        yCError(CCS) << "Could not retrieve parameters from ICartesianControl interface";
        return false;
    }

    for (const auto & [key, value] : params)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = false;

        switch (key)
        {
        case VOCAB_CC_CONFIG_STREAMING_CMD:
        {
            int intValue = static_cast<int>(value);
            std::string normalizedValue;

            switch (intValue)
            {
            case VOCAB_CC_POSE:
                normalizedValue = "pose";
                break;
            case VOCAB_CC_TWIST:
                normalizedValue = "twist";
                break;
            case VOCAB_CC_WRENCH:
                normalizedValue = "wrench";
                break;
            case VOCAB_CC_NOT_SET:
                normalizedValue = "none";
                break;
            default:
                yCError(CCS) << "Unknown preset streaming command value:" << intValue;
                return false;
            }

            descriptor.description = "Streaming command to be used by the device.";
            descriptor.additional_constraints = "Only 'pose', 'twist', 'wrench' or 'none' are allowed.";
            m_node->declare_parameter("preset_streaming_cmd", normalizedValue, descriptor);
            break;
        }
        case VOCAB_CC_CONFIG_FRAME:
        {
            int intValue = static_cast<int>(value);
            std::string normalizedValue;

            switch (intValue)
            {
            case ICartesianSolver::BASE_FRAME:
                normalizedValue = "base";
                break;
            case ICartesianSolver::TCP_FRAME:
                normalizedValue = "tcp";
                break;
            default:
                yCError(CCS) << "Unknown reference frame value:" << intValue;
                return false;
            }

            descriptor.description = "Reference frame to be used by the device.";
            descriptor.additional_constraints = "Only 'base' or 'tcp' are allowed.";
            m_node->declare_parameter("frame", normalizedValue, descriptor);
            break;
        }
        case VOCAB_CC_CONFIG_GAIN:
            descriptor.description = "Gain for the cartesian controller.";
            m_node->declare_parameter("gain", value, descriptor);
            break;
        case VOCAB_CC_CONFIG_TRAJ_DURATION:
            descriptor.description = "Default trajectory duration (seconds).";
            m_node->declare_parameter("trajectory_duration", value, descriptor);
            break;
        case VOCAB_CC_CONFIG_TRAJ_REF_SPD:
            descriptor.description = "Default trajectory reference speed (meters/second).";
            m_node->declare_parameter("trajectory_reference_speed", value, descriptor);
            break;
        case VOCAB_CC_CONFIG_TRAJ_REF_ACC:
            descriptor.description = "Default trajectory reference acceleration (meters/second^2).";
            m_node->declare_parameter("trajectory_reference_acceleration", value, descriptor);
            break;
        case VOCAB_CC_CONFIG_CMC_PERIOD:
            descriptor.description = "Cartesian controller period (seconds).";
            m_node->declare_parameter("cmc_period", value, descriptor);
            break;
        case VOCAB_CC_CONFIG_WAIT_PERIOD:
            descriptor.description = "Wait period for motion completion (seconds).";
            m_node->declare_parameter("wait_period", value, descriptor);
            break;
        }
    }

    m_params = m_node->add_on_set_parameters_callback(std::bind(&CartesianControlServerROS2::params_cb, this, std::placeholders::_1));

    return true;
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::destroyRosHandlers()
{
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

    m_params.reset();
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::pose_cb(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    const auto ori = KDL::Rotation::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    const auto rot = ori.GetRot();

    std::vector<double> v {
        msg->position.x, msg->position.y, msg->position.z,
        rot.x(), rot.y(), rot.z()
    };

    yCInfo(CCS) << "Received pose:" << v;
    m_iCartesianControl->pose(v);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::vector<double> v {
        msg->linear.x, msg->linear.y, msg->linear.z,
        msg->angular.x, msg->angular.y, msg->angular.z
    };

    yCInfo(CCS) << "Received twist:" << v;
    m_iCartesianControl->twist(v);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::wrench_cb(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    std::vector<double> v {
        msg->force.x, msg->force.y, msg->force.z,
        msg->torque.x, msg->torque.y, msg->torque.z
    };

    yCInfo(CCS) << "Received wrench:" << v;
    m_iCartesianControl->wrench(v);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::movj_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid movj command. Expected 6 elements.";
        return;
    }

    yCInfo(CCS) << "Received movj:" << msg->data;
    m_iCartesianControl->movj(msg->data);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::movl_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid movl command. Expected 6 elements.";
        return;
    }

    yCInfo(CCS) << "Received movl:" << msg->data;
    m_iCartesianControl->movl(msg->data);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::movv_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid movv command. Expected 6 elements.";
        return;
    }

    yCInfo(CCS) << "Received movv:" << msg->data;
    m_iCartesianControl->movv(msg->data);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::forc_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        yCError(CCS) << "Received invalid forc command. Expected 6 elements.";
        return;
    }

    yCInfo(CCS) << "Received forc:" << msg->data;
    m_iCartesianControl->forc(msg->data);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::tool_cb(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    const auto ori = KDL::Rotation::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    const auto rot = ori.GetRot();

    std::vector<double> v {
        msg->position.x, msg->position.y, msg->position.z,
        rot.x(), rot.y(), rot.z()
    };

    yCInfo(CCS) << "Received tool:" << v;
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
        pose.position.x, pose.position.y, pose.position.z,
        rot.x(), rot.y(), rot.z()
    };

    yCInfo(CCS) << "Received inv request:" << v;

    std::vector<double> q;
    response->success = m_iCartesianControl->inv(v, q);

    std::transform(q.begin(), q.end(), std::back_inserter(response->q.data),
                   [](double val) { return val * KDL::deg2rad; });

    response->q.data = q;
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::gcmp_cb(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
{
    yCInfo(CCS) << "Received gcmp request";
    response->success = m_iCartesianControl->gcmp();
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::stop_cb(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
{
    yCInfo(CCS) << "Received stop request";
    response->success = m_iCartesianControl->stopControl();
}

// -----------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult CartesianControlServerROS2::params_cb(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
    {
        const auto & name = param.get_name();
        int vocab;
        double value;

        if (name == "preset_streaming_cmd")
        {
            vocab = VOCAB_CC_CONFIG_STREAMING_CMD;

            if (const auto strValue = param.value_to_string(); strValue == "twist")
            {
                value = VOCAB_CC_TWIST;
            }
            else if (strValue == "pose")
            {
                value = VOCAB_CC_POSE;
            }
            else if (strValue == "wrench")
            {
                value = VOCAB_CC_WRENCH;
            }
            else if (strValue == "none")
            {
                value = VOCAB_CC_NOT_SET;
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid parameter value. Only 'twist', 'pose', 'wrench' or 'none' are allowed.";
                yCWarning(CCS) << "Invalid parameter value for:" << name;
                break;
            }
        }
        else if (name == "frame")
        {
            vocab = VOCAB_CC_CONFIG_FRAME;

            if (const auto strValue = param.value_to_string(); strValue == "base")
            {
                value = ICartesianSolver::BASE_FRAME;
            }
            else if (strValue == "tcp")
            {
                value = ICartesianSolver::TCP_FRAME;
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid parameter value. Only 'base' or 'tcp' are allowed.";
                yCWarning(CCS) << "Invalid parameter value for:" << name;
                break;
            }
        }
        else if (name == "gain")
        {
            vocab = VOCAB_CC_CONFIG_GAIN;
            value = param.as_double();
        }
        else if (name == "trajectory_duration")
        {
            vocab = VOCAB_CC_CONFIG_TRAJ_DURATION;
            value = param.as_double();
        }
        else if (name == "trajectory_reference_speed")
        {
            vocab = VOCAB_CC_CONFIG_TRAJ_REF_SPD;
            value = param.as_double();
        }
        else if (name == "trajectory_reference_acceleration")
        {
            vocab = VOCAB_CC_CONFIG_TRAJ_REF_ACC;
            value = param.as_double();
        }
        else if (name == "cmc_period")
        {
            vocab = VOCAB_CC_CONFIG_CMC_PERIOD;
            value = param.as_double();
        }
        else if (name == "wait_period")
        {
            vocab = VOCAB_CC_CONFIG_WAIT_PERIOD;
            value = param.as_double();
        }
        else
        {
            result.successful = false;
            result.reason = "Unexpected parameter name: " + name;
            yCWarning(CCS) << "Unexpected parameter name:" << name;
            break;
        }

        if (m_iCartesianControl->setParameter(vocab, value))
        {
            yCInfo(CCS) << "Parameter" << name << "set to" << param.value_to_string();
        }
        else
        {
            result.successful = false;
            result.reason = "Attached device's setParameter() method failed.";
            yCWarning(CCS) << "Could not set parameter:" << name;
            break;
        }
    }

    return result;
}

// -----------------------------------------------------------------------------

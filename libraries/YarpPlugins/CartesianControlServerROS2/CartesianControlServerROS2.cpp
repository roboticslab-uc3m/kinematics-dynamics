// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <algorithm> // std::transform
#include <utility> // std::invoke
#include <vector>

#include <kdl/frames.hpp>

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

namespace
{
    inline std::vector<double> pose_to_vector(const geometry_msgs::msg::Pose * msg)
    {
        auto rot = KDL::Rotation::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w).GetRot();

        return {
            msg->position.x, msg->position.y, msg->position.z,
            rot.x(), rot.y(), rot.z()
        };
    }

    inline std::vector<double> twist_to_vector(const geometry_msgs::msg::Twist * msg)
    {
        return {
            msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z
        };
    }

    inline std::vector<double> wrench_to_vector(const geometry_msgs::msg::Wrench * msg)
    {
        return {
            msg->force.x, msg->force.y, msg->force.z,
            msg->torque.x, msg->torque.y, msg->torque.z
        };
    }
}

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::configureRosHandlers()
{
    const auto prefix = "/" + m_name;

    m_state = m_node->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/state/pose", 10);

    m_pose = m_node->create_subscription<geometry_msgs::msg::Pose>(
        prefix + "/command/pose", 10,
        [this](geometry_msgs::msg::Pose::ConstSharedPtr msg)
        {
            const auto v = pose_to_vector(msg.get());
            yCDebug(CCS) << "Received pose command:" << v;
            m_iCartesianControl->pose(v);
        });

    if (!m_pose)
    {
        yCError(CCS) << "Could not initialize the pose command subscription";
        return false;
    }

    m_twist = m_node->create_subscription<geometry_msgs::msg::Twist>(
        prefix + "/command/twist", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg)
        {
            const auto v = twist_to_vector(msg.get());
            yCDebug(CCS) << "Received twist command:" << v;
            m_iCartesianControl->twist(twist_to_vector(msg.get()));
        });

    if (!m_twist)
    {
        yCError(CCS) << "Could not initialize the twist command subscription";
        return false;
    }

    m_wrench = m_node->create_subscription<geometry_msgs::msg::Wrench>(
        prefix + "/command/wrench", 10,
        [this](geometry_msgs::msg::Wrench::ConstSharedPtr msg)
        {
            const auto v = wrench_to_vector(msg.get());
            yCDebug(CCS) << "Received wrench command:" << v;
            m_iCartesianControl->wrench(v);
        });

    if (!m_wrench)
    {
        yCError(CCS) << "Could not initialize the wrench command subscription";
        return false;
    }

    m_movev = m_node->create_subscription<geometry_msgs::msg::Twist>(
        prefix + "/command/movev", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg)
        {
            const auto v = twist_to_vector(msg.get());
            yCDebug(CCS) << "Received movv command:" << v;
            m_iCartesianControl->moveVelocity(v);
        });

    if (!m_movev)
    {
        yCError(CCS) << "Could not initialize the movv command subscription";
        return false;
    }

    m_force = m_node->create_subscription<geometry_msgs::msg::Wrench>(
        prefix + "/command/force", 10,
        [this](geometry_msgs::msg::Wrench::ConstSharedPtr msg)
        {
            const auto v = wrench_to_vector(msg.get());
            yCDebug(CCS) << "Received forc command:" << v;
            m_iCartesianControl->forceControl(v);
        });

    if (!m_force)
    {
        yCError(CCS) << "Could not initialize the forc command subscription";
        return false;
    }

    m_tool = m_node->create_subscription<geometry_msgs::msg::Pose>(
        prefix + "/command/tool", 10,
        [this](geometry_msgs::msg::Pose::ConstSharedPtr msg)
        {
            const auto v = pose_to_vector(msg.get());
            yCDebug(CCS) << "Received tool command:" << v;
            m_iCartesianControl->changeTool(v);
        });

    if (!m_tool)
    {
        yCError(CCS) << "Could not initialize the tool command subscription";
        return false;
    }

    m_act = m_node->create_subscription<std_msgs::msg::Int32>(
        prefix + "/command/gripper", 10,
        [this](std_msgs::msg::Int32::ConstSharedPtr msg)
        {
            switch (msg->data)
            {
            case GRIPPER_CLOSE:
                yCDebug(CCS) << "Gripper close";
                m_iCartesianControl->actuateTool(ICartesianControl::Actuator::CLOSE);
                break;
            case GRIPPER_OPEN:
                yCDebug(CCS) << "Gripper open";
                m_iCartesianControl->actuateTool(ICartesianControl::Actuator::OPEN);
                break;
            case GRIPPER_STOP:
                yCDebug(CCS) << "Gripper stop";
                m_iCartesianControl->actuateTool(ICartesianControl::Actuator::STOP);
                break;
            }
        });

    if (!m_act)
    {
        yCError(CCS) << "Could not initialize the gripper command subscription";
        return false;
    }

    m_inv = m_node->create_service<rl_cartesian_control_msgs::srv::Inv>(
        prefix + "/inv",
        [this](const rl_cartesian_control_msgs::srv::Inv::Request::SharedPtr request, rl_cartesian_control_msgs::srv::Inv::Response::SharedPtr response)
        {
            std::vector<double> q;
            response->success = m_iCartesianControl->solvePose(pose_to_vector(&request->x), q);
            std::transform(q.begin(), q.end(), std::back_inserter(response->q.data), [](double val) { return val * KDL::deg2rad; });
            response->q.data = q;
        });

    if (!m_inv)
    {
        yCError(CCS) << "Could not initialize the inv service";
        return false;
    }

    m_gcmp = m_node->create_service<std_srvs::srv::Trigger>(
        prefix + "/gcmp",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            yCDebug(CCS) << "Received gcmp request";
            response->success = m_iCartesianControl->gravityCompensation();
        });

    if (!m_gcmp)
    {
        yCError(CCS) << "Could not initialize the gcmp service";
        return false;
    }

    m_stop = m_node->create_service<std_srvs::srv::Trigger>(
        prefix + "/stop",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            yCDebug(CCS) << "Received stop request";
            response->success = m_iCartesianControl->stopControl();

            if (m_goalHandle && m_goalHandle->is_active())
            {
                yCDebug(CCS) << "Canceling active trajectory goal due to stop request";

                ICartesianControl::ControllerState state;
                m_iCartesianControl->getState(state);

                auto result_msg = std::make_shared<rl_cartesian_control_msgs::action::Trajectory::Result>();
                result_msg->success = true; // not a failure, just a user-requested stop
                result_msg->duration = state.duration;
                result_msg->progress = state.progress;

                m_goalHandle->canceled(result_msg);
            }
        });

    if (!m_stop)
    {
        yCError(CCS) << "Could not initialize the stop service";
        return false;
    }

    m_trajectory = rclcpp_action::create_server<rl_cartesian_control_msgs::action::Trajectory>(
        m_node,
        prefix + "/trajectory",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const rl_cartesian_control_msgs::action::Trajectory::Goal> goal)
        {
            yCDebug(CCS) << "Received trajectory goal request";

            if (m_goalHandle && m_goalHandle->is_active())
            {
                yCError(CCS) << "A trajectory goal is already active, rejecting new goal";
                return rclcpp_action::GoalResponse::REJECT;
            }

            using command_t = yarp::dev::ReturnValue (ICartesianControl::*)(const std::vector<double> &);
            command_t command = nullptr;

            switch (goal->type)
            {
            case rl_cartesian_control_msgs::action::Trajectory::Goal::JOINT:
                yCDebug(CCS) << "Trajectory type: JOINT";
                command = &ICartesianControl::moveJoint;
                break;
            case rl_cartesian_control_msgs::action::Trajectory::Goal::LINEAR:
                yCDebug(CCS) << "Trajectory type: LINEAR";
                command = &ICartesianControl::moveLinear;
                break;
            default:
                yCError(CCS) << "Unknown trajectory type:" << goal->type;
                return rclcpp_action::GoalResponse::REJECT;
            }

            return std::invoke(command, m_iCartesianControl, pose_to_vector(&goal->x))
                ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                : rclcpp_action::GoalResponse::REJECT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rl_cartesian_control_msgs::action::Trajectory>> goalHandle)
        {
            yCDebug(CCS) << "Received trajectory cancel request";

            return m_iCartesianControl->stopControl()
                ? rclcpp_action::CancelResponse::ACCEPT
                : rclcpp_action::CancelResponse::REJECT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rl_cartesian_control_msgs::action::Trajectory>> goalHandle)
        {
            yCDebug(CCS) << "Executing trajectory goal";
            m_goalHandle = goalHandle;
        });

    if (!m_trajectory)
    {
        yCError(CCS) << "Could not initialize the trajectory action server";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::configureRosParameters()
{
    std::map<ICartesianControl::Config, double> params;

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
        case ICartesianControl::Config::STREAMING_CMD:
        {
            auto vocab = static_cast<ICartesianControl::Streaming>(value);
            std::string normalizedValue;

            switch (vocab)
            {
            case ICartesianControl::Streaming::POSE:
                normalizedValue = "pose";
                break;
            case ICartesianControl::Streaming::TWIST:
                normalizedValue = "twist";
                break;
            case ICartesianControl::Streaming::WRENCH:
                normalizedValue = "wrench";
                break;
            default:
                if (value == static_cast<double>(ICartesianControl::Vocabs::NOT_SET))
                {
                    normalizedValue = "none";
                }
                else
                {
                    yCError(CCS) << "Unknown preset streaming command value:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(value));
                    return false;
                }
            }

            descriptor.description = "Streaming command to be used by the device.";
            descriptor.additional_constraints = "Only 'pose', 'twist', 'wrench' or 'none' are allowed.";
            m_node->declare_parameter("preset_streaming_cmd", normalizedValue, descriptor);
            break;
        }
        case ICartesianControl::Config::FRAME:
        {
            auto vocab = static_cast<ICartesianSolver::Frame>(value);
            std::string normalizedValue;

            switch (vocab)
            {
            case ICartesianSolver::Frame::BASE:
                normalizedValue = "base";
                break;
            case ICartesianSolver::Frame::TCP:
                normalizedValue = "tcp";
                break;
            default:
                yCError(CCS) << "Unknown reference frame value:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(value));
                return false;
            }

            descriptor.description = "Reference frame to be used by the device.";
            descriptor.additional_constraints = "Only 'base' or 'tcp' are allowed.";
            m_node->declare_parameter("frame", normalizedValue, descriptor);
            break;
        }
        case ICartesianControl::Config::GAIN:
            descriptor.description = "Gain for the cartesian controller.";
            m_node->declare_parameter("gain", value, descriptor);
            break;
        case ICartesianControl::Config::TRAJ_DURATION:
            descriptor.description = "Default trajectory duration (seconds).";
            m_node->declare_parameter("trajectory_duration", value, descriptor);
            break;
        case ICartesianControl::Config::TRAJ_REF_SPD:
            descriptor.description = "Default trajectory reference speed (meters/second).";
            m_node->declare_parameter("trajectory_reference_speed", value, descriptor);
            break;
        case ICartesianControl::Config::TRAJ_REF_ACC:
            descriptor.description = "Default trajectory reference acceleration (meters/second^2).";
            m_node->declare_parameter("trajectory_reference_acceleration", value, descriptor);
            break;
        case ICartesianControl::Config::CMC_PERIOD:
            descriptor.description = "Cartesian controller period (seconds).";
            m_node->declare_parameter("cmc_period", value, descriptor);
            break;
        case ICartesianControl::Config::WAIT_PERIOD:
            descriptor.description = "Wait period for motion completion (seconds).";
            m_node->declare_parameter("wait_period", value, descriptor);
            break;
        }
    }

    m_params = m_node->add_on_set_parameters_callback([this](const auto & parameters) { return params_cb(parameters); });

    if (!m_params)
    {
        yCError(CCS) << "Could not register parameter callback";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::destroyRosHandlers()
{
    m_state.reset();
    m_movev.reset();
    m_force.reset();
    m_tool.reset();
    m_act.reset();
    m_pose.reset();
    m_twist.reset();
    m_wrench.reset();

    m_inv.reset();
    m_gcmp.reset();
    m_stop.reset();

    m_trajectory.reset();
    m_goalHandle.reset();

    m_params.reset();
}

// -----------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult CartesianControlServerROS2::params_cb(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
    {
        const auto & name = param.get_name();
        ICartesianControl::Config vocab;
        double value;

        if (name == "preset_streaming_cmd")
        {
            vocab = ICartesianControl::Config::STREAMING_CMD;

            if (const auto strValue = param.value_to_string(); strValue == "twist")
            {
                value = static_cast<double>(ICartesianControl::Streaming::TWIST);
            }
            else if (strValue == "pose")
            {
                value = static_cast<double>(ICartesianControl::Streaming::POSE);
            }
            else if (strValue == "wrench")
            {
                value = static_cast<double>(ICartesianControl::Streaming::WRENCH);
            }
            else if (strValue == "none")
            {
                value = static_cast<double>(ICartesianControl::Vocabs::NOT_SET);
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
            vocab = ICartesianControl::Config::FRAME;

            if (const auto strValue = param.value_to_string(); strValue == "base")
            {
                value = static_cast<double>(ICartesianSolver::Frame::BASE);
            }
            else if (strValue == "tcp")
            {
                value = static_cast<double>(ICartesianSolver::Frame::TCP);
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
            vocab = ICartesianControl::Config::GAIN;
            value = param.as_double();
        }
        else if (name == "trajectory_duration")
        {
            vocab = ICartesianControl::Config::TRAJ_DURATION;
            value = param.as_double();
        }
        else if (name == "trajectory_reference_speed")
        {
            vocab = ICartesianControl::Config::TRAJ_REF_SPD;
            value = param.as_double();
        }
        else if (name == "trajectory_reference_acceleration")
        {
            vocab = ICartesianControl::Config::TRAJ_REF_ACC;
            value = param.as_double();
        }
        else if (name == "cmc_period")
        {
            vocab = ICartesianControl::Config::CMC_PERIOD;
            value = param.as_double();
        }
        else if (name == "wait_period")
        {
            vocab = ICartesianControl::Config::WAIT_PERIOD;
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

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
    inline std::vector<double> pose_to_vector(const geometry_msgs::msg::Pose & msg)
    {
        auto rot = KDL::Rotation::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w).GetRot();

        return {
            msg.position.x, msg.position.y, msg.position.z,
            rot.x(), rot.y(), rot.z()
        };
    }

    inline std::vector<double> twist_to_vector(const geometry_msgs::msg::Twist & msg)
    {
        return {
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        };
    }

    inline std::vector<double> wrench_to_vector(const geometry_msgs::msg::Wrench & msg)
    {
        return {
            msg.force.x, msg.force.y, msg.force.z,
            msg.torque.x, msg.torque.y, msg.torque.z
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
            const auto v = pose_to_vector(*msg);
            yCDebug(CCS) << "Received pose command:" << v;
            m_iCartesianControl->pose(v);
        });

    m_twist = m_node->create_subscription<geometry_msgs::msg::Twist>(
        prefix + "/command/twist", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg)
        {
            const auto v = twist_to_vector(*msg);
            yCDebug(CCS) << "Received twist command:" << v;
            m_iCartesianControl->twist(v);
        });

    m_wrench = m_node->create_subscription<geometry_msgs::msg::Wrench>(
        prefix + "/command/wrench", 10,
        [this](geometry_msgs::msg::Wrench::ConstSharedPtr msg)
        {
            const auto v = wrench_to_vector(*msg);
            yCDebug(CCS) << "Received wrench command:" << v;
            m_iCartesianControl->wrench(v);
        });

    m_move_v = m_node->create_service<rl_cartesian_control_msgs::srv::MoveV>(
        prefix + "/move_velocity",
        [this](const rl_cartesian_control_msgs::srv::MoveV::Request::SharedPtr request, rl_cartesian_control_msgs::srv::MoveV::Response::SharedPtr response)
        {
            const auto v = twist_to_vector(request->xdot);
            yCDebug(CCS) << "Received velocity move request:" << v;
            response->success = m_iCartesianControl->moveVelocity(v);
        });

    m_force = m_node->create_service<rl_cartesian_control_msgs::srv::Force>(
        prefix + "/force_control",
        [this](const rl_cartesian_control_msgs::srv::Force::Request::SharedPtr request, rl_cartesian_control_msgs::srv::Force::Response::SharedPtr response)
        {
            const auto v = wrench_to_vector(request->f);
            yCDebug(CCS) << "Received force control request:" << v;
            response->success = m_iCartesianControl->forceControl(v);
        });

    m_tool = m_node->create_service<rl_cartesian_control_msgs::srv::Tool>(
        prefix + "/change_tool",
        [this](const rl_cartesian_control_msgs::srv::Tool::Request::SharedPtr request, rl_cartesian_control_msgs::srv::Tool::Response::SharedPtr response)
        {
            const auto v = pose_to_vector(request->x);
            yCDebug(CCS) << "Received change tool request:" << v;
            response->success = m_iCartesianControl->changeTool(v);
        });

    if (!m_tool)
    {
        yCError(CCS) << "Could not initialize the change tool service";
        return false;
    }

    m_inv = m_node->create_service<rl_cartesian_control_msgs::srv::Inv>(
        prefix + "/solve_pose",
        [this](const rl_cartesian_control_msgs::srv::Inv::Request::SharedPtr request, rl_cartesian_control_msgs::srv::Inv::Response::SharedPtr response)
        {
            const auto v = pose_to_vector(request->x);
            yCDebug(CCS) << "Received solve pose request:" << v;
            std::vector<double> q;
            response->success = m_iCartesianControl->solvePose(v, q);
            std::transform(q.begin(), q.end(), std::back_inserter(response->q), [](double val) { return val * KDL::deg2rad; });
        });

    m_act = m_node->create_service<rl_cartesian_control_msgs::srv::Act>(
        prefix + "/actuate_tool",
        [this](const rl_cartesian_control_msgs::srv::Act::Request::SharedPtr request, rl_cartesian_control_msgs::srv::Act::Response::SharedPtr response)
        {
            yarp::dev::ReturnValue ret;

            switch (request->cmd)
            {
            case rl_cartesian_control_msgs::srv::Act::Request::CLOSE:
                yCDebug(CCS) << "Received actuate tool (close) request";
                ret = m_iCartesianControl->actuateTool(ICartesianControl::Actuator::CLOSE);
                break;
            case rl_cartesian_control_msgs::srv::Act::Request::OPEN:
                yCDebug(CCS) << "Received actuate tool (open) request";
                ret = m_iCartesianControl->actuateTool(ICartesianControl::Actuator::OPEN);
                break;
            case rl_cartesian_control_msgs::srv::Act::Request::STOP:
                yCDebug(CCS) << "Received actuate tool (stop) request";
                ret = m_iCartesianControl->actuateTool(ICartesianControl::Actuator::STOP);
                break;
            }

            response->success = ret;
        });

    m_gcmp = m_node->create_service<std_srvs::srv::Trigger>(
        prefix + "/gravity_compensation",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            yCDebug(CCS) << "Received gravity compensation request";
            response->success = m_iCartesianControl->gravityCompensation();
        });

    m_stop = m_node->create_service<std_srvs::srv::Trigger>(
        prefix + "/stop_control",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            yCDebug(CCS) << "Received stop control request";
            response->success = m_iCartesianControl->stopControl();
        });

    m_trajectory = rclcpp_action::create_server<rl_cartesian_control_msgs::action::PoseTrajectory>(
        m_node,
        prefix + "/trajectory/pose",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const rl_cartesian_control_msgs::action::PoseTrajectory::Goal> goal)
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
            case rl_cartesian_control_msgs::action::PoseTrajectory::Goal::JOINT:
                yCDebug(CCS) << "Trajectory type: JOINT";
                command = &ICartesianControl::moveJoint;
                break;
            case rl_cartesian_control_msgs::action::PoseTrajectory::Goal::LINEAR:
                yCDebug(CCS) << "Trajectory type: LINEAR";
                command = &ICartesianControl::moveLinear;
                break;
            default:
                yCError(CCS) << "Unknown trajectory type:" << goal->type;
                return rclcpp_action::GoalResponse::REJECT;
            }

            return std::invoke(command, m_iCartesianControl, pose_to_vector(goal->pose))
                ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                : rclcpp_action::GoalResponse::REJECT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rl_cartesian_control_msgs::action::PoseTrajectory>> goalHandle)
        {
            yCDebug(CCS) << "Received trajectory cancel request";

            return m_iCartesianControl->stopControl()
                ? rclcpp_action::CancelResponse::ACCEPT
                : rclcpp_action::CancelResponse::REJECT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rl_cartesian_control_msgs::action::PoseTrajectory>> goalHandle)
        {
            yCDebug(CCS) << "Executing trajectory goal";
            m_goalHandle = goalHandle;
        });

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

    return true;
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::destroyRosHandlers()
{
    m_state.reset();

    m_pose.reset();
    m_twist.reset();
    m_wrench.reset();

    m_move_v.reset();
    m_force.reset();
    m_tool.reset();
    m_inv.reset();
    m_act.reset();

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

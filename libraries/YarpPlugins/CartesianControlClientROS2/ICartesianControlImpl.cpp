// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include <algorithm> // std::all_of
#include <unordered_map>

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include <kdl/frames.hpp>

#include "LogComponent.hpp"

using namespace roboticslab;

namespace
{
    const std::unordered_map<ICartesianControl::Config, std::string> vocabToParamName = {
        {ICartesianControl::Config::GAIN, "gain"},
        {ICartesianControl::Config::TRAJ_DURATION, "trajectory_duration"},
        {ICartesianControl::Config::TRAJ_REF_SPD, "trajectory_reference_speed"},
        {ICartesianControl::Config::TRAJ_REF_ACC, "trajectory_reference_acceleration"},
        {ICartesianControl::Config::CMC_PERIOD, "cmc_period"},
        {ICartesianControl::Config::WAIT_PERIOD, "wait_period"},
        {ICartesianControl::Config::FRAME, "frame"},
        {ICartesianControl::Config::STREAMING_CMD, "preset_streaming_cmd"}
    };

    bool parameterFromVocab(ICartesianControl::Config vocab, double value, rcl_interfaces::msg::Parameter & param)
    {
        switch (vocab)
        {
        case ICartesianControl::Config::GAIN:
        case ICartesianControl::Config::TRAJ_DURATION:
        case ICartesianControl::Config::TRAJ_REF_SPD:
        case ICartesianControl::Config::TRAJ_REF_ACC:
        case ICartesianControl::Config::CMC_PERIOD:
        case ICartesianControl::Config::WAIT_PERIOD:
            param.value.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
            param.value.double_value = value;
            break;
        case ICartesianControl::Config::FRAME:
            param.value.type = rclcpp::ParameterType::PARAMETER_STRING;

            switch (static_cast<ICartesianSolver::Frame>(value))
            {
            case ICartesianSolver::Frame::BASE:
                param.value.string_value = "base";
                break;
            case ICartesianSolver::Frame::TCP:
                param.value.string_value = "tcp";
                break;
            default:
                yCError(CCC) << "Invalid frame";
                return false;
            }

            break;
        case ICartesianControl::Config::STREAMING_CMD:
            param.value.type = rclcpp::ParameterType::PARAMETER_STRING;

            switch (static_cast<ICartesianControl::Streaming>(value))
            {
            case ICartesianControl::Streaming::POSE:
                param.value.string_value = "pose";
                break;
            case ICartesianControl::Streaming::TWIST:
                param.value.string_value = "twist";
                break;
            case ICartesianControl::Streaming::WRENCH:
                param.value.string_value = "wrench";
                break;
            default:
                if (value == static_cast<double>(ICartesianControl::Vocabs::NOT_SET))
                {
                    param.value.string_value = "none";
                }
                else
                {
                    yCError(CCC) << "Invalid streaming command";
                    return false;
                }
            }

            break;
        default:
            yCError(CCC) << "Invalid parameter vocab:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
            return false;
        }

        param.name = vocabToParamName.at(vocab);

        return true;
    }

    bool vocabFromParameter(const std::string & name, const rcl_interfaces::msg::ParameterValue & paramValue, ICartesianControl::Config * vocab, double * value)
    {
        if (name == "gain")
        {
            *vocab = ICartesianControl::Config::GAIN;
            *value = paramValue.double_value;
        }
        else if (name == "trajectory_duration")
        {
            *vocab = ICartesianControl::Config::TRAJ_DURATION;
            *value = paramValue.double_value;
        }
        else if (name == "trajectory_reference_speed")
        {
            *vocab = ICartesianControl::Config::TRAJ_REF_SPD;
            *value = paramValue.double_value;
        }
        else if (name == "trajectory_reference_acceleration")
        {
            *vocab = ICartesianControl::Config::TRAJ_REF_ACC;
            *value = paramValue.double_value;
        }
        else if (name == "cmc_period")
        {
            *vocab = ICartesianControl::Config::CMC_PERIOD;
            *value = paramValue.double_value;
        }
        else if (name == "wait_period")
        {
            *vocab = ICartesianControl::Config::WAIT_PERIOD;
            *value = paramValue.double_value;
        }
        else if (name == "frame")
        {
            *vocab = ICartesianControl::Config::FRAME;

            if (paramValue.string_value == "base")
            {
                *value = static_cast<double>(ICartesianSolver::Frame::BASE);
            }
            else if (paramValue.string_value == "tcp")
            {
                *value = static_cast<double>(ICartesianSolver::Frame::TCP);
            }
            else
            {
                yCError(CCC) << "Invalid parameter value for:" << name;
                return false;
            }
        }
        else if (name == "preset_streaming_cmd")
        {
            *vocab = ICartesianControl::Config::STREAMING_CMD;

            if (paramValue.string_value == "pose")
            {
                *value = static_cast<double>(ICartesianControl::Streaming::POSE);
            }
            else if (paramValue.string_value == "twist")
            {
                *value = static_cast<double>(ICartesianControl::Streaming::TWIST);
            }
            else if (paramValue.string_value == "wrench")
            {
                *value = static_cast<double>(ICartesianControl::Streaming::WRENCH);
            }
            else if (paramValue.string_value == "none")
            {
                *value = static_cast<double>(ICartesianControl::Vocabs::NOT_SET);
            }
            else
            {
                yCError(CCC) << "Invalid parameter value for:" << name;
                return false;
            }
        }
        else
        {
            // silence logs here since this function is called for every parameter in the response,
            // and we only care about the ones we support
            return false;
        }

        return true;
    }
}

// ------------------- ICartesianControl Related ------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::stat(std::vector<double> & x, State * state, double * timestamp)
{
    std::lock_guard lock(m_mutex_state);

    KDL::Vector axisAngle = KDL::Rotation::Quaternion(
        m_pose_last.pose.orientation.x,
        m_pose_last.pose.orientation.y,
        m_pose_last.pose.orientation.z,
        m_pose_last.pose.orientation.w
    ).GetRot();

    x = {
        m_pose_last.pose.position.x, m_pose_last.pose.position.y, m_pose_last.pose.position.z,
        axisAngle.x(), axisAngle.y(), axisAngle.z()
    };

    if (timestamp)
    {
        *timestamp = m_pose_last.header.stamp.sec + m_pose_last.header.stamp.nanosec * 1e-9;
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::inv(const std::vector<double> & xd, std::vector<double> & q)
{
    rl_cartesian_control_msgs::srv::Inv::Request request;
    request.x.position.x = xd[0];
    request.x.position.y = xd[1];
    request.x.position.z = xd[2];

    KDL::Vector axis(xd[3], xd[4], xd[5]);
    double angle = axis.Norm();

    KDL::Rotation::Rot(axis, angle).GetQuaternion(
        request.x.orientation.x,
        request.x.orientation.y,
        request.x.orientation.z,
        request.x.orientation.w
    );

    auto result = m_client_inv->async_send_request(std::make_shared<rl_cartesian_control_msgs::srv::Inv::Request>(request));
    auto response = result.get();

    if (!response->success)
    {
        yCError(CCC) << "Inverse kinematics service call failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    q = response->q.data;
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::movj(const std::vector<double> & xd)
{
    geometry_msgs::msg::Pose poseMsg;
    poseMsg.position.x = xd[0];
    poseMsg.position.y = xd[1];
    poseMsg.position.z = xd[2];

    KDL::Vector axis(xd[3], xd[4], xd[5]);
    double angle = axis.Norm();

    KDL::Rotation::Rot(axis, angle).GetQuaternion(
        poseMsg.orientation.x,
        poseMsg.orientation.y,
        poseMsg.orientation.z,
        poseMsg.orientation.w
    );

    m_movj->publish(poseMsg);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::relj(const std::vector<double> & xd)
{
    geometry_msgs::msg::Pose poseMsg;
    poseMsg.position.x = xd[0];
    poseMsg.position.y = xd[1];
    poseMsg.position.z = xd[2];

    KDL::Vector axis(xd[3], xd[4], xd[5]);
    double angle = axis.Norm();

    KDL::Rotation::Rot(axis, angle).GetQuaternion(
        poseMsg.orientation.x,
        poseMsg.orientation.y,
        poseMsg.orientation.z,
        poseMsg.orientation.w
    );

    m_relj->publish(poseMsg);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::movl(const std::vector<double> & xd)
{
    geometry_msgs::msg::Pose poseMsg;
    poseMsg.position.x = xd[0];
    poseMsg.position.y = xd[1];
    poseMsg.position.z = xd[2];

    KDL::Vector axis(xd[3], xd[4], xd[5]);
    double angle = axis.Norm();

    KDL::Rotation::Rot(axis, angle).GetQuaternion(
        poseMsg.orientation.x,
        poseMsg.orientation.y,
        poseMsg.orientation.z,
        poseMsg.orientation.w
    );

    m_movl->publish(poseMsg);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::movv(const std::vector<double> & xdotd)
{
    geometry_msgs::msg::Twist twistMsg;
    twistMsg.linear.x = xdotd[0];
    twistMsg.linear.y = xdotd[1];
    twistMsg.linear.z = xdotd[2];
    twistMsg.angular.x = xdotd[3];
    twistMsg.angular.y = xdotd[4];
    twistMsg.angular.z = xdotd[5];

    m_movv->publish(twistMsg);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::gcmp()
{
    std_srvs::srv::Trigger::Request request;
    auto result = m_client_gcmp->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(request));
    auto response = result.get();

    return response->success
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::forc(const std::vector<double> & fd)
{
    geometry_msgs::msg::Wrench wrenchMsg;
    wrenchMsg.force.x = fd[0];
    wrenchMsg.force.y = fd[1];
    wrenchMsg.force.z = fd[2];
    wrenchMsg.torque.x = fd[3];
    wrenchMsg.torque.y = fd[4];
    wrenchMsg.torque.z = fd[5];

    m_forc->publish(wrenchMsg);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::stopControl()
{
    std_srvs::srv::Trigger::Request request;
    auto result = m_client_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(request));
    auto response = result.get();

    return response->success
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::wait(double timeout)
{
    yCWarning(CCC) << "wait() not implemented for CartesianControlClientROS2";
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::tool(const std::vector<double> & x)
{
    geometry_msgs::msg::Pose poseMsg;
    poseMsg.position.x = x[0];
    poseMsg.position.y = x[1];
    poseMsg.position.z = x[2];

    KDL::Vector axis(x[3], x[4], x[5]);
    double angle = axis.Norm();

    KDL::Rotation::Rot(axis, angle).GetQuaternion(
        poseMsg.orientation.x,
        poseMsg.orientation.y,
        poseMsg.orientation.z,
        poseMsg.orientation.w
    );

    m_tool->publish(poseMsg);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::act(Actuator command)
{
    std_msgs::msg::Int32 actMsg;

    switch (command)
    {
    case Actuator::NONE:
        actMsg.data = GRIPPER_NONE;
        break;
    case Actuator::OPEN:
        actMsg.data = GRIPPER_OPEN;
        break;
    case Actuator::CLOSE:
        actMsg.data = GRIPPER_CLOSE;
        break;
    case Actuator::STOP:
        actMsg.data = GRIPPER_STOP;
        break;
    default:
        yCError(CCC) << "Invalid actuator command:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(command));
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    m_act->publish(actMsg);
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::pose(const std::vector<double> & x)
{
    geometry_msgs::msg::Pose poseMsg;
    poseMsg.position.x = x[0];
    poseMsg.position.y = x[1];
    poseMsg.position.z = x[2];

    KDL::Vector axis(x[3], x[4], x[5]);
    double angle = axis.Norm();

    KDL::Rotation::Rot(axis, angle).GetQuaternion(
        poseMsg.orientation.x,
        poseMsg.orientation.y,
        poseMsg.orientation.z,
        poseMsg.orientation.w
    );

    m_pose->publish(poseMsg);
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::twist(const std::vector<double> & xdot)
{
    geometry_msgs::msg::Twist twistMsg;
    twistMsg.linear.x = xdot[0];
    twistMsg.linear.y = xdot[1];
    twistMsg.linear.z = xdot[2];
    twistMsg.angular.x = xdot[3];
    twistMsg.angular.y = xdot[4];
    twistMsg.angular.z = xdot[5];

    m_twist->publish(twistMsg);
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::wrench(const std::vector<double> & w)
{
    geometry_msgs::msg::Wrench wrenchMsg;
    wrenchMsg.force.x = w[0];
    wrenchMsg.force.y = w[1];
    wrenchMsg.force.z = w[2];
    wrenchMsg.torque.x = w[3];
    wrenchMsg.torque.y = w[4];
    wrenchMsg.torque.z = w[5];

    m_wrench->publish(wrenchMsg);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::setParameter(Config vocab, double value)
{
    rcl_interfaces::msg::Parameter param;

    if (!parameterFromVocab(vocab, value, param))
    {
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(param);

    auto result = m_client_set_params->async_send_request(request);
    auto response = result.get();

    return response->results.size() == 1 && response->results[0].successful
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::getParameter(Config vocab, double * value)
{
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    if (vocabToParamName.find(vocab) == vocabToParamName.end())
    {
        yCError(CCC) << "Invalid parameter vocab:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    const auto & name = vocabToParamName.at(vocab);
    request->names.push_back(name);

    auto result = m_client_get_params->async_send_request(request);
    auto response = result.get();

    if (response->values.size() != 1)
    {
        yCError(CCC) << "Unexpected number of parameter values received";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return vocabFromParameter(name, response->values[0], &vocab, value)
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::setParameters(const std::map<Config, double> & params)
{
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    for (const auto & [vocab, value] : params)
    {
        rcl_interfaces::msg::Parameter param;

        if (!parameterFromVocab(vocab, value, param))
        {
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }

        request->parameters.push_back(param);
    }

    auto result = m_client_set_params->async_send_request(request);
    auto response = result.get();
    const auto & results = response->results;

    return std::all_of(results.begin(), results.end(), [](const auto & r) { return r.successful; })
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClientROS2::getParameters(std::map<Config, double> & params)
{
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = m_supported_parameters;

    auto result = m_client_get_params->async_send_request(request);
    auto response = result.get();

    for (auto i = 0; i < response->values.size(); ++i)
    {
        const auto & name = request->names[i];
        const auto & responseValue = response->values[i];

        Config vocab;
        double value;

        if (vocabFromParameter(name, responseValue, &vocab, &value))
        {
            params[vocab] = value;
        }
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

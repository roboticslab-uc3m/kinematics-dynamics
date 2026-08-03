// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClientROS2.hpp"

#include <algorithm> // std::all_of
#include <unordered_map>

#include <yarp/os/LogStream.h>

#include <kdl/frames.hpp>

#include "LogComponent.hpp"

namespace
{
    const std::unordered_map<int, std::string> vocabToParamName = {
        {VOCAB_CC_CONFIG_GAIN, "gain"},
        {VOCAB_CC_CONFIG_TRAJ_DURATION, "trajectory_duration"},
        {VOCAB_CC_CONFIG_TRAJ_REF_SPD, "trajectory_reference_speed"},
        {VOCAB_CC_CONFIG_TRAJ_REF_ACC, "trajectory_reference_acceleration"},
        {VOCAB_CC_CONFIG_CMC_PERIOD, "cmc_period"},
        {VOCAB_CC_CONFIG_WAIT_PERIOD, "wait_period"},
        {VOCAB_CC_CONFIG_FRAME, "frame"},
        {VOCAB_CC_CONFIG_STREAMING_CMD, "preset_streaming_cmd"}
    };

    bool parameterFromVocab(int vocab, double value, rcl_interfaces::msg::Parameter & param)
    {
        switch (vocab)
        {
        case VOCAB_CC_CONFIG_GAIN:
        case VOCAB_CC_CONFIG_TRAJ_DURATION:
        case VOCAB_CC_CONFIG_TRAJ_REF_SPD:
        case VOCAB_CC_CONFIG_TRAJ_REF_ACC:
        case VOCAB_CC_CONFIG_CMC_PERIOD:
        case VOCAB_CC_CONFIG_WAIT_PERIOD:
            param.value.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
            param.value.double_value = value;
            break;
        case VOCAB_CC_CONFIG_FRAME:
            param.value.type = rclcpp::ParameterType::PARAMETER_STRING;

            switch (static_cast<int>(value))
            {
            case roboticslab::ICartesianSolver::BASE_FRAME:
                param.value.string_value = "base";
                break;
            case roboticslab::ICartesianSolver::TCP_FRAME:
                param.value.string_value = "tcp";
                break;
            default:
                yCError(CCC) << "Invalid frame";
                return false;
            }

            break;
        case VOCAB_CC_CONFIG_STREAMING_CMD:
            param.value.type = rclcpp::ParameterType::PARAMETER_STRING;

            switch (static_cast<int>(value))
            {
            case VOCAB_CC_POSE:
                param.value.string_value = "pose";
                break;
            case VOCAB_CC_TWIST:
                param.value.string_value = "twist";
                break;
            case VOCAB_CC_WRENCH:
                param.value.string_value = "wrench";
                break;
            case VOCAB_CC_NOT_SET:
                param.value.string_value = "none";
                break;
            default:
                yCError(CCC) << "Invalid streaming command";
                return false;
            }

            break;
        default:
            yCError(CCC) << "Invalid parameter vocab:" << vocab;
            return false;
        }

        param.name = vocabToParamName.at(vocab);

        return true;
    }

    bool vocabFromParameter(const std::string & name, const rcl_interfaces::msg::ParameterValue & paramValue, int * vocab, double * value)
    {
        if (name == "gain")
        {
            *vocab = VOCAB_CC_CONFIG_GAIN;
            *value = paramValue.double_value;
        }
        else if (name == "trajectory_duration")
        {
            *vocab = VOCAB_CC_CONFIG_TRAJ_DURATION;
            *value = paramValue.double_value;
        }
        else if (name == "trajectory_reference_speed")
        {
            *vocab = VOCAB_CC_CONFIG_TRAJ_REF_SPD;
            *value = paramValue.double_value;
        }
        else if (name == "trajectory_reference_acceleration")
        {
            *vocab = VOCAB_CC_CONFIG_TRAJ_REF_ACC;
            *value = paramValue.double_value;
        }
        else if (name == "cmc_period")
        {
            *vocab = VOCAB_CC_CONFIG_CMC_PERIOD;
            *value = paramValue.double_value;
        }
        else if (name == "wait_period")
        {
            *vocab = VOCAB_CC_CONFIG_WAIT_PERIOD;
            *value = paramValue.double_value;
        }
        else if (name == "frame")
        {
            *vocab = VOCAB_CC_CONFIG_FRAME;

            if (paramValue.string_value == "base")
            {
                *value = static_cast<double>(roboticslab::ICartesianSolver::BASE_FRAME);
            }
            else if (paramValue.string_value == "tcp")
            {
                *value = static_cast<double>(roboticslab::ICartesianSolver::TCP_FRAME);
            }
            else
            {
                yCError(CCC) << "Invalid parameter value for:" << name;
                return false;
            }
        }
        else if (name == "preset_streaming_cmd")
        {
            *vocab = VOCAB_CC_CONFIG_STREAMING_CMD;

            if (paramValue.string_value == "pose")
            {
                *value = static_cast<double>(VOCAB_CC_POSE);
            }
            else if (paramValue.string_value == "twist")
            {
                *value = static_cast<double>(VOCAB_CC_TWIST);
            }
            else if (paramValue.string_value == "wrench")
            {
                *value = static_cast<double>(VOCAB_CC_WRENCH);
            }
            else if (paramValue.string_value == "none")
            {
                *value = static_cast<double>(VOCAB_CC_NOT_SET);
            }
            else
            {
                yCError(CCC) << "Invalid parameter value for:" << name;
                return false;
            }
        }
        else
        {
            yCError(CCC) << "Invalid parameter name:" << name;
            return false;
        }

        return true;
    }
}

// ------------------- ICartesianControl Related ------------------------------------

bool CartesianControlClientROS2::stat(std::vector<double> & x, int * state, double * timestamp)
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

    *timestamp = m_pose_last.header.stamp.sec + m_pose_last.header.stamp.nanosec * 1e-9;

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::inv(const std::vector<double> & xd, std::vector<double> & q)
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
        return false;
    }

    q = response->q.data;
    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movj(const std::vector<double> & xd)
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

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::relj(const std::vector<double> & xd)
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

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movl(const std::vector<double> & xd)
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

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movv(const std::vector<double> & xdotd)
{
    geometry_msgs::msg::Twist twistMsg;
    twistMsg.linear.x = xdotd[0];
    twistMsg.linear.y = xdotd[1];
    twistMsg.linear.z = xdotd[2];
    twistMsg.angular.x = xdotd[3];
    twistMsg.angular.y = xdotd[4];
    twistMsg.angular.z = xdotd[5];

    m_movv->publish(twistMsg);

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::gcmp()
{
    std_srvs::srv::Trigger::Request request;
    auto result = m_client_gcmp->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(request));
    auto response = result.get();
    return response->success;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::forc(const std::vector<double> & fd)
{
    geometry_msgs::msg::Wrench wrenchMsg;
    wrenchMsg.force.x = fd[0];
    wrenchMsg.force.y = fd[1];
    wrenchMsg.force.z = fd[2];
    wrenchMsg.torque.x = fd[3];
    wrenchMsg.torque.y = fd[4];
    wrenchMsg.torque.z = fd[5];

    m_forc->publish(wrenchMsg);

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::stopControl()
{
    std_srvs::srv::Trigger::Request request;
    auto result = m_client_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(request));
    auto response = result.get();
    return response->success;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::wait(double timeout)
{
    yCWarning(CCC) << "wait() not implemented for CartesianControlClientROS2";
    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::tool(const std::vector<double> & x)
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

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::act(int command)
{
    std_msgs::msg::Int32 actMsg;
    actMsg.data = command;
    m_act->publish(actMsg);
    return true;
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::pose(const std::vector<double> & x)
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
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::twist(const std::vector<double> & xdot)
{
    geometry_msgs::msg::Twist twistMsg;
    twistMsg.linear.x = xdot[0];
    twistMsg.linear.y = xdot[1];
    twistMsg.linear.z = xdot[2];
    twistMsg.angular.x = xdot[3];
    twistMsg.angular.y = xdot[4];
    twistMsg.angular.z = xdot[5];

    m_twist->publish(twistMsg);
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::wrench(const std::vector<double> & w)
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

bool CartesianControlClientROS2::setParameter(int vocab, double value)
{
    rcl_interfaces::msg::Parameter param;

    if (!parameterFromVocab(vocab, value, param))
    {
        return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(param);

    auto result = m_client_set_params->async_send_request(request);
    auto response = result.get();

    return response->results.size() == 1 && response->results[0].successful;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::getParameter(int vocab, double * value)
{
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    if (vocabToParamName.find(vocab) == vocabToParamName.end())
    {
        yCError(CCC) << "Invalid parameter vocab:" << vocab;
        return false;
    }

    const auto & name = vocabToParamName.at(vocab);
    request->names.push_back(name);

    auto result = m_client_get_params->async_send_request(request);
    auto response = result.get();

    if (response->values.size() != 1)
    {
        yCError(CCC) << "Unexpected number of parameter values received";
        return false;
    }

    return vocabFromParameter(name, response->values[0], &vocab, value);
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::setParameters(const std::map<int, double> & params)
{
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    for (const auto & [vocab, value] : params)
    {
        rcl_interfaces::msg::Parameter param;

        if (!parameterFromVocab(vocab, value, param))
        {
            return false;
        }

        request->parameters.push_back(param);
    }

    auto result = m_client_set_params->async_send_request(request);
    auto response = result.get();
    const auto & results = response->results;

    return std::all_of(results.begin(), results.end(), [](const auto & r) { return r.successful; });
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::getParameters(std::map<int, double> & params)
{
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    for (const auto & [vocab, _] : params)
    {
        if (vocabToParamName.find(vocab) == vocabToParamName.end())
        {
            yCError(CCC) << "Invalid parameter vocab:" << vocab;
            return false;
        }

        request->names.push_back(vocabToParamName.at(vocab));
    }

    auto result = m_client_get_params->async_send_request(request);
    auto response = result.get();
    const auto & values = response->values;

    if (values.size() != params.size())
    {
        yCError(CCC) << "Mismatch in number of parameters received";
        return false;
    }

    for (auto i = 0; i < values.size(); ++i)
    {
        const auto & name = request->names[i];
        const auto & responseValue = values[i];

        int vocab;
        double value;

        if (!vocabFromParameter(name, responseValue, &vocab, &value))
        {
            return false;
        }

        params[vocab] = value;
    }

    return true;
}

// -----------------------------------------------------------------------------

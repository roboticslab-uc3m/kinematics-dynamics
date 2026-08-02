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
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movj(const std::vector<double> & xd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::relj(const std::vector<double> & xd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movl(const std::vector<double> & xd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::movv(const std::vector<double> & xdotd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::gcmp()
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::forc(const std::vector<double> & fd)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::stopControl()
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::wait(double timeout)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::tool(const std::vector<double> & x)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CartesianControlClientROS2::act(int command)
{
    return false;
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::pose(const std::vector<double> & x)
{
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::twist(const std::vector<double> & xdot)
{
}

// -----------------------------------------------------------------------------

void CartesianControlClientROS2::wrench(const std::vector<double> & w)
{
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

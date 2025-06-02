// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <string>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlServerROS2::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(CCS) << "Failed to parse parameters";
        return false;
    }

    if (m_fkPeriod > 0)
    {
        yarp::os::PeriodicThread::setPeriod(m_fkPeriod * 0.001);
    }
    else
    {
        yCWarning(CCS) << "Invalid period, using default";
        yarp::os::PeriodicThread::setPeriod(std::stoi(m_fkPeriod_defaultValue) * 0.001);
    }

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // ROS2 initialization
    m_node = std::make_shared<rclcpp::Node>(m_name);

    rcl_interfaces::msg::ParameterDescriptor descriptor_streaming_cmd;
    descriptor_streaming_cmd.name = "preset_streaming_cmd";
    descriptor_streaming_cmd.description = "Streaming command to be used by the device.";
    descriptor_streaming_cmd.read_only = false;
    descriptor_streaming_cmd.additional_constraints = "Only 'pose', 'twist', 'wrench' or 'none' are allowed.";
    descriptor_streaming_cmd.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING);

    m_node->declare_parameter("preset_streaming_cmd", "none", descriptor_streaming_cmd);
    preset_streaming_cmd = VOCAB_CC_NOT_SET;

    rcl_interfaces::msg::ParameterDescriptor descriptor_frame;
    descriptor_frame.name = "frame";
    descriptor_frame.description = "Reference frame to be used by the device.";
    descriptor_frame.read_only = false;
    descriptor_frame.additional_constraints = "Only 'base' or 'tcp' are allowed.";
    descriptor_frame.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING);

    m_node->declare_parameter<std::string>("frame", "base", descriptor_frame);
    frame = ICartesianSolver::BASE_FRAME;

    // Spin node for ROS2
    m_spinner = new Spinner(m_node);

    if (!m_spinner)
    {
        yCError(CCS) << "Failed to create spinner";
        return false;
    }

    return m_spinner->start();
}

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::close()
{
    if (m_spinner && m_spinner->isRunning())
    {
        return m_spinner->stop();
    }

    return true;
}

// -----------------------------------------------------------------------------

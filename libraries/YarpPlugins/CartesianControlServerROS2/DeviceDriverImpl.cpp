// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlServerROS2::open(yarp::os::Searchable & config)
{
    // No es genérico, cambiar antes de entregar

    yCInfo(CCS) << "Subdevice wrapping";

    // maybe user isn't doing nested configuration
    yarp::os::Property p;
    p.fromString(config.toString());
    p.put("device", "BasicCartesianControl");
    cartesianControlDevice.open(p);

    if (!cartesianControlDevice.isValid())
    {
        yCError(CCS) << "Cannot make";
    }

    if (!cartesianControlDevice.isValid())
    {
        yCError(CCS) << "Cartesian control device not valid";
        return false;
    }

    if (!cartesianControlDevice.view(m_iCartesianControl))
    {
        yCError(CCS) << "Problems acquiring ICartesianControl interface";
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

    auto prefix = "/" + m_name;

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // ROS2 initialization
    m_node = std::make_shared<rclcpp::Node>(m_name);

    // Parameters
    callback_handle_ = m_node->add_on_set_parameters_callback(std::bind(&CartesianControlServerROS2::parameter_callback, this, std::placeholders::_1));

    rcl_interfaces::msg::ParameterDescriptor descriptor_msg;
    descriptor_msg.name = "preset_streaming_cmd";
    descriptor_msg.description = "Streaming command to be used by the device.";
    descriptor_msg.read_only = false;
    descriptor_msg.additional_constraints = "Only 'twist', 'pose', 'wrench' or 'none' are allowed.";
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING);

    m_node->declare_parameter("preset_streaming_cmd", "none", descriptor_msg); //null default value
    m_node->get_parameter("preset_streaming_cmd", preset_streaming_cmd);

    rcl_interfaces::msg::ParameterDescriptor descriptor_frame;
    descriptor_frame.name = "frame";
    descriptor_frame.description = "Reference frame to be used by the device.";
    descriptor_frame.read_only = false;
    descriptor_frame.additional_constraints = "Only 'base' or 'tcp' are allowed.";
    descriptor_frame.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING);

    m_node->declare_parameter<std::string>("frame", "base", descriptor_frame); // Default frame (base)
    m_node->get_parameter("frame", frame);

    // Publisher
    m_publisher = m_node->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/state/pose", 10);

    // Subscriptions
    m_poseSubscription = m_node->create_subscription<geometry_msgs::msg::Pose>(prefix + "/command/pose", 10,
                                                                               std::bind(&CartesianControlServerROS2::poseTopic_callback,
                                                                               this, std::placeholders::_1));
    if (!m_poseSubscription)
    {
        yCError(CCS) << "Could not initialize the Pose msg subscription";
        return false;
    }

    m_twistSubscription = m_node->create_subscription<geometry_msgs::msg::Twist>(prefix + "/command/twist", 10,
                                                                                 std::bind(&CartesianControlServerROS2::twistTopic_callback,
                                                                                 this, std::placeholders::_1));
    if (!m_twistSubscription)
    {
        yCError(CCS) << "Could not initialize the Twist msg subscription";
        return false;
    }

    m_wrenchSubscription = m_node->create_subscription<geometry_msgs::msg::Wrench>(prefix + "/command/wrench", 10,
                                                                                   std::bind(&CartesianControlServerROS2::wrenchTopic_callback,
                                                                                   this, std::placeholders::_1));
    if (!m_wrenchSubscription)
    {
        yCError(CCS) << "Could not initialize the Wrench msg subscription";
        return false;
    }

    m_gripperSubscription = m_node->create_subscription<std_msgs::msg::Int32>(prefix + "/command/gripper", 10,
                                                                              std::bind(&CartesianControlServerROS2::gripperTopic_callback,
                                                                              this, std::placeholders::_1));

    if (!m_gripperSubscription)
    {
        yCError(CCS) << "Could not initialize the Gripper msg subscription";
        return false;
    }

        m_movj = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(prefix + "/command/movjoint", 10,
                                                                                   std::bind(&CartesianControlServerROS2::movj_callback,
                                                                                             this, std::placeholders::_1));
    if(!m_movj)
    {
        yCError(CCS) << "Could not initialize the movj msg subscription";
        return false;
    }

    m_movl = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(prefix + "/command/movlinear", 10,
                                                                           std::bind(&CartesianControlServerROS2::movl_callback,
                                                                                     this, std::placeholders::_1));
    if(!m_movl)
    {
        yCError(CCS) << "Could not initialize the movl msg subscription";
        return false;
    }

    // Spin node for ROS2
    m_spinner = new Spinner(m_node);

    if (!m_spinner)
    {
        yCError(CCS) << "Failed to create spinner";
        return false;
    }

    return m_spinner->start() && yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::close()
{
    if (m_spinner && m_spinner->isRunning())
    {
        m_spinner->stop();
    }

    return cartesianControlDevice.close();
}

// ------------------- Spinner Class Related ------------------------------------

Spinner::Spinner(std::shared_ptr<rclcpp::Node> input_node)
    : m_node(input_node)
{}

// -----------------------------------------------------------------------------

Spinner::~Spinner()
{
    if (m_spun)
    {
        rclcpp::shutdown();
        m_spun = false;
    }
}

// -----------------------------------------------------------------------------

void Spinner::run()
{
    if (!m_spun)
    {
        m_spun = true;
        rclcpp::spin(m_node);
    }
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Spinner.hpp"

using namespace roboticslab::ros2utils;

// -----------------------------------------------------------------------------

rclcpp::Node::SharedPtr createNode(const std::string & name,
                                   const std::string & ns,
                                   const rclcpp::NodeOptions & options)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    return std::make_shared<rclcpp::Node>(name, ns, options);
}

// ------------------- Spinner Class Related ------------------------------------

Spinner::Spinner(rclcpp::Node::SharedPtr node)
    : m_node(node)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
}

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

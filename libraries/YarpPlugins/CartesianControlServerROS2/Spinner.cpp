// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Spinner.hpp"

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

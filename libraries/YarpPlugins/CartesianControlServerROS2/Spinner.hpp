// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SPINNER_HPP__
#define __SPINNER_HPP__

#include <yarp/os/Thread.h>

#include <rclcpp/rclcpp.hpp>

class Spinner : public yarp::os::Thread
{
public:
    Spinner(std::shared_ptr<rclcpp::Node> input_node);
    ~Spinner() override;
    void run() override;

private:
    bool m_spun {false};
    std::shared_ptr<rclcpp::Node> m_node;
};

#endif // __SPINNER_HPP__

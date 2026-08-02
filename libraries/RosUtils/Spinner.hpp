// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SPINNER_HPP__
#define __SPINNER_HPP__

#include <memory>
#include <string>

#include <yarp/os/Thread.h>

#include <rclcpp/rclcpp.hpp>

namespace roboticslab
{

namespace ros2utils
{

rclcpp::Node::SharedPtr createNode(const std::string & name,
                                   const std::string & ns = "", // namespace
                                   const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

class Spinner : public yarp::os::Thread
{
public:
    Spinner(rclcpp::Node::SharedPtr node);
    ~Spinner() override;
    void run() override;

    using Ptr = std::unique_ptr<Spinner>;

private:
    bool m_spun {false};
    rclcpp::Node::SharedPtr m_node;
};

} // namespace ros2utils

} // namespace roboticslab

#endif // __SPINNER_HPP__

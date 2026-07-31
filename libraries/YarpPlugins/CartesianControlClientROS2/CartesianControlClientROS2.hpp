// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__
#define __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__

#include <yarp/dev/Drivers.h>

#include <rclcpp/rclcpp.hpp>

#include "ICartesianControl.h"
#include "CartesianControlClientROS2_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup CartesianControlClientROS2
 *
 * @brief Contains CartesianControlClientROS2.
 */

/**
 * @ingroup CartesianControlClientROS2
 * @brief The CartesianControlClientROS2 class implements ICartesianControl client side.
 */
class CartesianControlClientROS2 : public yarp::dev::DeviceDriver,
                                   public roboticslab::ICartesianControl,
                                   public CartesianControlClientROS2_ParamsParser
{
public:
    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --
    bool stat(std::vector<double> & x, int * state = nullptr, double * timestamp = nullptr) override;
    bool inv(const std::vector<double> & xd, std::vector<double> & q) override;
    bool movj(const std::vector<double> & xd) override;
    bool relj(const std::vector<double> & xd) override;
    bool movl(const std::vector<double> & xd) override;
    bool movv(const std::vector<double> & xdotd) override;
    bool gcmp() override;
    bool forc(const std::vector<double> & fd) override;
    bool stopControl() override;
    bool wait(double timeout) override;
    bool tool(const std::vector<double> & x) override;
    bool act(int command) override;
    void pose(const std::vector<double> & x) override;
    void twist(const std::vector<double> & xdot) override;
    void wrench(const std::vector<double> & w) override;
    bool setParameter(int vocab, double value) override;
    bool getParameter(int vocab, double * value) override;
    bool setParameters(const std::map<int, double> & params) override;
    bool getParameters(std::map<int, double> & params) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    rclcpp::Node::SharedPtr m_node;
};

#endif // __CARTESIAN_CONTROL_CLIENT_ROS2_HPP__

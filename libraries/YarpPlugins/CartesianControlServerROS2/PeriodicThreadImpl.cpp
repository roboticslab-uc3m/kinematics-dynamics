// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <cmath> // std::modf

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- PeriodicThread related ------------------------------------

void CartesianControlServerROS2::run()
{
    std::vector<double> x;
    int state;
    double timestamp;

    if (!m_iCartesianControl->stat(x, &state, &timestamp))
    {
        yCWarning(CCS) << "Failed to get status";
        return;
    }

    double sec;
    double nsec = std::modf(timestamp, &sec) * 1e9;

    const auto rot = KDL::Vector(x[3], x[4], x[5]);
    const auto ori = KDL::Rotation::Rot(rot, rot.Norm());

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nsec;
    msg.pose.position.x = x[0];
    msg.pose.position.y = x[1];
    msg.pose.position.z = x[2];

    ori.GetQuaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);

    m_publisher->publish(msg);
}

// -----------------------------------------------------------------------------

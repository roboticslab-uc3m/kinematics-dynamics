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
    ICartesianControl::ControllerState state;

    if (!m_iCartesianControl->getState(state))
    {
        yCWarning(CCS) << "Failed to getState";
        return;
    }

    double sec;
    double nsec = std::modf(state.timestamp, &sec) * 1e9;

    const auto rot = KDL::Vector(state.x[3], state.x[4], state.x[5]);
    const auto ori = KDL::Rotation::Rot(rot, rot.Norm());

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nsec;

    msg.pose.position.x = state.x[0];
    msg.pose.position.y = state.x[1];
    msg.pose.position.z = state.x[2];

    ori.GetQuaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);

    m_state->publish(msg);

    if (m_goalHandle && m_goalHandle->is_executing())
    {
        auto result_msg = std::make_shared<rl_cartesian_control_msgs::action::Trajectory::Result>();

        if (m_goalHandle->is_canceling())
        {
            // TODO: this can also happen if stopControl() is called or the internal controller decides to stop by itself
            yCInfo(CCS) << "Trajectory goal canceled";
            m_goalHandle->canceled(result_msg);
        }
        else if (state.mode == ICartesianControl::Mode::NONE)
        {
            yCInfo(CCS) << "Trajectory goal succeeded";
            m_goalHandle->succeed(result_msg);
        }
        else
        {
            // TODO: handle progress feedback
            // m_goalHandle->publish_feedback(progress);
        }
    }
}

// -----------------------------------------------------------------------------

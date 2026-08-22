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

    geometry_msgs::msg::PoseStamped state_msg;
    state_msg.header.stamp.sec = sec;
    state_msg.header.stamp.nanosec = nsec;

    state_msg.pose.position.x = state.x[0];
    state_msg.pose.position.y = state.x[1];
    state_msg.pose.position.z = state.x[2];

    ori.GetQuaternion(state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w);

    m_state->publish(state_msg);

    if (m_goalHandle)
    {
        using Trajectory = rl_cartesian_control_msgs::action::Trajectory;
        using Mode = ICartesianControl::Mode;

        if (m_goalHandle->is_executing())
        {
            if (m_goalHandle->get_goal()->type == Trajectory::Goal::JOINT && state.mode == Mode::MOVEJ ||
                m_goalHandle->get_goal()->type == Trajectory::Goal::LINEAR && state.mode == Mode::MOVEL)
            {
                auto feedback_msg = std::make_shared<Trajectory::Feedback>();
                feedback_msg->progress = state.progress;
                m_goalHandle->publish_feedback(feedback_msg);
            }
            else
            {
                yCInfo(CCS) << "Trajectory execution finished, success:" << state.success;
                auto result_msg = std::make_shared<Trajectory::Result>();
                result_msg->success = state.success;
                state.success ? m_goalHandle->succeed(result_msg) : m_goalHandle->abort(result_msg);
            }
        }
        else if (m_goalHandle->is_canceling())
        {
            yCInfo(CCS) << "Trajectory execution canceled";
            auto result_msg = std::make_shared<Trajectory::Result>();
            result_msg->success = true; // not a failure, just a user-requested cancel
            m_goalHandle->canceled(result_msg);
        }
    }
}

// -----------------------------------------------------------------------------

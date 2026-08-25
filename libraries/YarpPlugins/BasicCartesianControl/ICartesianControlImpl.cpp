// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <algorithm> // std::transform
#include <functional> // std::negate
#include <iterator> // std::back_inserter
#include <limits>
#include <vector>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- ICartesianControl Related ------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::getState(roboticslab::ICartesianControl::ControllerState & state)
{
    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCErrorThreadThrottle(BCC, 1.0) << "getEncoders() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    if (!iCartesianSolver->forwardKinematics(currentQ, state.x))
    {
        yCError(BCC) << "forwardKinematics() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    state.mode = currentMode;
    state.timestamp = getTimestamp();
    state.duration = maxTrajectoryDuration;
    state.progress = cmcProgress;
    state.success = cmcSuccess;

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::solvePose(const std::vector<double> & xd, std::vector<double> & q)
{
    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    if (!iCartesianSolver->inverseKinematics(xd, currentQ, q, referenceFrame))
    {
        yCError(BCC) << "inverseKinematics() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::moveJoint(const std::vector<double> & xd)
{
    std::vector<double> currentQ(numJoints), qd;

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    if (!iCartesianSolver->inverseKinematics(xd, currentQ, qd, referenceFrame))
    {
        yCError(BCC) << "inverseKinematics() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    if (std::vector<double> vmo(numJoints); computeIsocronousSpeeds(currentQ, qd, vmo))
    {
        vmoStored.resize(numJoints);

#if YARP_VERSION_COMPARE(>=, 4,0,0)
        if (!iPositionControl->getTrajSpeeds(vmoStored.data()))
        {
            yCError(BCC) << "getTrajSpeeds() (for storing) failed";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }

        if (!iPositionControl->setTrajSpeeds(vmo.data()))
        {
            yCError(BCC) << "setTrajSpeeds() failed";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }
#else
        if (!iPositionControl->getRefSpeeds(vmoStored.data()))
        {
            yCError(BCC) << "getRefSpeeds() (for storing) failed";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }

        if (!iPositionControl->setRefSpeeds(vmo.data()))
        {
            yCError(BCC) << "setRefSpeeds() failed";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }
#endif

        if (!setControlModes(VOCAB_CM_POSITION))
        {
            yCError(BCC) << "Unable to set position mode";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }

        if (!iPositionControl->positionMove(qd.data()))
        {
            yCError(BCC) << "positionMove() failed";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }

        trajectoryStartTime = getTimestamp();
        cmcSuccess = true;
        cmcProgress = 0.0f;
        yCInfo(BCC) << "Performing MOVEJ";

        currentMode = Mode::MOVEJ;
    }
    else
    {
        yCWarning(BCC) << "No motion planned";
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::moveLinear(const std::vector<double> & xd)
{
    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->forwardKinematics(currentQ, x_base_tcp))
    {
        yCError(BCC) << "forwardKinematics() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    std::vector<double> xd_obj;

    if (referenceFrame == ICartesianSolver::Frame::TCP)
    {
        if (!iCartesianSolver->changeOrigin(xd, x_base_tcp, xd_obj))
        {
            yCError(BCC) << "changeOrigin() failed";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }
    }
    else
    {
        xd_obj = xd;
    }

    trajectories.clear();

    maxTrajectoryDuration = std::numeric_limits<double>::max();

    //-- Create line trajectories (one per endpoint if robot is a kin-tree)
    for (unsigned int i = 0; i < xd.size() / 6; i++)
    {
        std::vector<double> xd_base_tcp_sub(x_base_tcp.cbegin() + i * 6, x_base_tcp.cbegin() + (i + 1) * 6);
        std::vector<double> xd_obj_sub(xd_obj.cbegin() + i * 6, xd_obj.cbegin() + (i + 1) * 6);

        auto H_base_start = KdlVectorConverter::vectorToFrame(xd_base_tcp_sub);
        auto H_base_end = KdlVectorConverter::vectorToFrame(xd_obj_sub);

        auto * interpolator = new KDL::RotationalInterpolation_SingleAxis();
        auto * path = new KDL::Path_Line(H_base_start, H_base_end, interpolator, 1.0);
        auto * profile = new KDL::VelocityProfile_Trap(m_trajectoryRefSpeed, m_trajectoryRefAccel);

        if (m_trajectoryDuration != 0.0)
        {
            // Set duration, let profile compute speed and acceleration
            profile->SetProfileDuration(0.0, path->PathLength(), m_trajectoryDuration);
        }
        else
        {
            // Set speed and acceleration, let profile compute duration
            profile->SetProfile(0.0, path->PathLength());
        }

        auto duration = profile->Duration();

        if (duration < maxTrajectoryDuration)
        {
            maxTrajectoryDuration = duration;
        }

        trajectories.emplace_back(new KDL::Trajectory_Segment(path, profile));
    }

    if (m_enableFailFast && !doFailFastChecks(currentQ))
    {
        yCError(BCC) << "Fail-fast checks failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    if (!setControlModes(m_usePosdMovel ? VOCAB_CM_POSITION_DIRECT : VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Unable to set" << (m_usePosdMovel ? "position direct" : "velocity") << "control mode";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    trajectoryStartTime = getTimestamp();
    cmcSuccess = true;
    cmcProgress = 0.0f;
    yCInfo(BCC) << "Performing MOVEL";

    currentMode = Mode::MOVEL;

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::moveVelocity(const std::vector<double> & xdotd)
{
    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->forwardKinematics(currentQ, x_base_tcp))
    {
        yCError(BCC) << "forwardKinematics() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    trajectories.clear();

    maxTrajectoryDuration = std::numeric_limits<double>::max();

    for (unsigned int i = 0; i < xdotd.size() / 6; i++)
    {
        std::vector<double> xd_base_tcp_sub(x_base_tcp.cbegin() + i * 6, x_base_tcp.cbegin() + (i + 1) * 6);
        std::vector<double> xdotd_sub(xdotd.cbegin() + i * 6, xdotd.cbegin() + (i + 1) * 6);

        auto H_base_start = KdlVectorConverter::vectorToFrame(xd_base_tcp_sub);
        auto twist_in_base = KdlVectorConverter::vectorToTwist(xdotd_sub);

        auto * interpolator = new KDL::RotationalInterpolation_SingleAxis();
        auto * path = new KDL::Path_Line(H_base_start, twist_in_base, interpolator, 1.0);
        auto * profile = new KDL::VelocityProfile_Rectangular(m_trajectoryRefSpeed);
        profile->SetProfileDuration(0.0, m_trajectoryRefSpeed, m_trajectoryRefSpeed / path->PathLength());

        auto duration = profile->Duration();

        if (duration < maxTrajectoryDuration)
        {
            maxTrajectoryDuration = duration;
        }

        trajectories.emplace_back(new KDL::Trajectory_Segment(path, profile));
    }

    //-- Set velocity mode and set state which makes periodic thread implement control
    if (!setControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Unable to set velocity mode";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    trajectoryStartTime = getTimestamp();
    cmcSuccess = true;
    yCInfo(BCC) << "Performing MOVEV";

    currentMode = Mode::MOVEV;

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::gravityCompensation()
{
    if (!setControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Unable to set torque mode";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    currentMode = Mode::GCMP;

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::forceControl(const std::vector<double> & fd)
{
    yCWarning(BCC) << "FORCE mode still experimental";

    if (referenceFrame == ICartesianSolver::Frame::TCP)
    {
        yCWarning(BCC) << "TCP frame not supported yet in forc command";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    this->fd.clear();

    // negate since the solver's contract interprets the wrench as an external force applied on
    // the end-effector, while the controller's contract interprets it as a force exerted by us
    std::transform(fd.cbegin(), fd.cend(), std::back_inserter(this->fd), std::negate<>());

    if (!setControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Unable to set torque mode";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    currentMode = Mode::FORCE;

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::stopControl()
{
    yCDebug(BCC) << "Stopping control";

    currentMode = Mode::NONE;

    // first switch control so that manipulators don't fall due to e.g. gravity
    if (!setControlModes(VOCAB_CM_POSITION))
    {
        yCWarning(BCC) << "setControlModes(VOCAB_CM_POSITION) failed";
    }

    // stop joints if already controlling position
    if (!iPositionControl->stop())
    {
        yCWarning(BCC) << "stop() failed";
    }

    trajectories.clear();

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::changeTool(const std::vector<double> & x)
{
    if (!iCartesianSolver->restoreOriginalChain())
    {
        yCError(BCC) << "restoreOriginalChain() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    if (!iCartesianSolver->appendLink(x))
    {
        yCError(BCC) << "appendLink() failed";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::actuateTool(Actuator command)
{
    yCError(BCC) << "act() not implemented";
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::pose(const std::vector<double> & x)
{
    if (currentMode != Mode::NONE ||
        streamingCommand != Streaming::POSE ||
        !checkControlModes(VOCAB_CM_POSITION_DIRECT))
    {
        yCError(BCC) << "Streaming command not preset";
        return;
    }

    std::vector<double> currentQ(numJoints), q;

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return;
    }

    if (!iCartesianSolver->inverseKinematics(x, currentQ, q, referenceFrame))
    {
        yCError(BCC) << "inverseKinematics() failed";
        return;
    }

    std::vector<double> qdiff(numJoints);

    for (int i = 0; i < numJoints; i++)
    {
        qdiff[i] = q[i] - currentQ[i];
    }

    if (!checkJointLimits(currentQ, qdiff))
    {
        yCError(BCC) << "Joint position limits exceeded, not moving";
        return;
    }

    if (!iPositionDirect->setPositions(q.data()))
    {
        yCError(BCC) << "setPositions() failed";
        return;
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::twist(const std::vector<double> & xdot)
{
    if (currentMode != Mode::NONE ||
        streamingCommand != Streaming::TWIST ||
        !checkControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Streaming command not preset";
        return;
    }

    StateWatcher watcher([this] { iVelocityControl->velocityMove(std::vector(numJoints, 0.0).data()); });
    std::vector<double> currentQ(numJoints), qdot;

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return;
    }

    if (!iCartesianSolver->diffInverseKinematics(currentQ, xdot, qdot, referenceFrame))
    {
        yCError(BCC) << "diffInverseKinematics() failed";
        return;
    }

    if (!checkJointLimits(currentQ, qdot) || !checkJointVelocities(qdot))
    {
        yCError(BCC) << "Joint position or velocity limits exceeded";
        return;
    }

    watcher.suppress();

    if (!iVelocityControl->velocityMove(qdot.data()))
    {
        yCError(BCC) << "velocityMove() failed";
        return;
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::wrench(const std::vector<double> & w)
{
    if (currentMode != Mode::NONE ||
        streamingCommand != Streaming::WRENCH ||
        !checkControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Streaming command not preset";
        return;
    }

    StateWatcher watcher([this] { iTorqueControl->setRefTorques(std::vector(numJoints, 0.0).data()); });
    std::vector<double> currentQ(numJoints), currentQdot(numJoints), currentQdotdot(numJoints), ftip;

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return;
    }

    if (!iEncoders->getEncoderSpeeds(currentQdot.data()))
    {
        yCError(BCC) << "getEncoderSpeeds() failed";
        return;
    }

    if (!iEncoders->getEncoderAccelerations(currentQdotdot.data()))
    {
        yCError(BCC) << "getEncoderAccelerations() failed";
        return;
    }

    if (!checkJointLimits(currentQ))
    {
        yCError(BCC) << "Joint position limits exceeded, not moving";
        return;
    }

    // negate since the solver's contract interprets the wrench as an external force applied on
    // the end-effector, while the controller's contract interprets it as a force exerted by us
    std::transform(w.cbegin(), w.cend(), std::back_inserter(ftip), std::negate<>());

    std::vector<double> t;

    if (!iCartesianSolver->inverseDynamics(currentQ, currentQdot, currentQdotdot, ftip, t, referenceFrame))
    {
        yCError(BCC) << "inverseDynamics() failed";
        return;
    }

    watcher.suppress();

    if (!iTorqueControl->setRefTorques(t.data()))
    {
        yCError(BCC) << "setRefTorques() failed";
        return;
    }
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::setParameter(Config vocab, double value)
{
    if (currentMode != Mode::NONE)
    {
        yCError(BCC) << "Unable to set config parameter while controlling";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    switch (vocab)
    {
    case Config::GAIN:
        if (value < 0.0)
        {
            yCError(BCC) << "Controller gain cannot be negative";
#if YARP_VERSION_COMPARE(>=, 4,0,0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
        m_controllerGain = value;
        break;
    case Config::TRAJ_DURATION:
        if (value < 0.0)
        {
            yCError(BCC) << "Trajectory duration cannot be negative";
#if YARP_VERSION_COMPARE(>=, 4,0,0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
        else if ((m_trajectoryDuration == 0.0) ^ (value == 0.0))
        {
            if (value == 0.0)
            {
                yCInfo(BCC) << "Duration set to zero, therefore trajectory execution time will depend on reference speed and acceleration";
            }
            else
            {
                yCInfo(BCC) << "Trajectory duration forced to" << value << "seconds regardless of velocity profile";
            }
        }
        m_trajectoryDuration = value;
        break;
    case Config::TRAJ_REF_SPD:
        if (value <= 0.0)
        {
            yCError(BCC) << "Trajectory reference speed cannot be negative nor zero";
#if YARP_VERSION_COMPARE(>=, 4,0,0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
        m_trajectoryRefSpeed = value;
        break;
    case Config::TRAJ_REF_ACC:
        if (value <= 0.0)
        {
            yCError(BCC) << "Trajectory reference acceleration cannot be negative nor zero";
#if YARP_VERSION_COMPARE(>=, 4,0,0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
        m_trajectoryRefAccel = value;
        break;
    case Config::CMC_PERIOD:
        if (!yarp::os::PeriodicThread::setPeriod(value * 0.001))
        {
            yCError(BCC) << "Cannot set new CMC period";
#if YARP_VERSION_COMPARE(>=, 4,0,0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
        m_cmcPeriodMs = value;
        break;
    case Config::WAIT_PERIOD:
        if (value <= 0.0)
        {
            yCError(BCC) << "Wait period cannot be negative nor zero";
#if YARP_VERSION_COMPARE(>=, 4,0,0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
        m_waitPeriodMs = value;
        break;
    case Config::FRAME:
        if (value != static_cast<double>(ICartesianSolver::Frame::BASE) &&
            value != static_cast<double>(ICartesianSolver::Frame::TCP))
        {
            yCError(BCC) << "Unrecognized or unsupported reference frame vocab";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }
        referenceFrame = static_cast<ICartesianSolver::Frame>(value);
        break;
    case Config::STREAMING_CMD:
        if (!presetStreamingCommand(static_cast<Streaming>(value)))
        {
            yCError(BCC) << "Unable to preset streaming command";
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }
        streamingCommand = static_cast<Streaming>(value);
        break;
    default:
        yCError(BCC) << "Unrecognized or unsupported config parameter key:"
                     << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::getParameter(Config vocab, double * value)
{
    switch (vocab)
    {
    case Config::GAIN:
        *value = m_controllerGain;
        break;
    case Config::TRAJ_DURATION:
        *value = m_trajectoryDuration;
        break;
    case Config::TRAJ_REF_SPD:
        *value = m_trajectoryRefSpeed;
        break;
    case Config::TRAJ_REF_ACC:
        *value = m_trajectoryRefAccel;
        break;
    case Config::CMC_PERIOD:
        *value = m_cmcPeriodMs;
        break;
    case Config::WAIT_PERIOD:
        *value = m_waitPeriodMs;
        break;
    case Config::FRAME:
        *value = static_cast<double>(referenceFrame);
        break;
    case Config::STREAMING_CMD:
        *value = static_cast<double>(streamingCommand);
        break;
    default:
        yCError(BCC) << "Unrecognized or unsupported config parameter key:"
                     << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::setParameters(const std::map<Config, double> & params)
{
    if (currentMode != Mode::NONE)
    {
        yCError(BCC) << "Unable to set config parameters while controlling";
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    bool ok = true;

    for (const auto & [vocab, value] : params)
    {
        ok &= setParameter(vocab, value);
    }

    return ok ? yarp::dev::ReturnValue::return_code::return_value_ok
              : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue BasicCartesianControl::getParameters(std::map<Config, double> & params)
{
    params.emplace(Config::GAIN, m_controllerGain);
    params.emplace(Config::TRAJ_DURATION, m_trajectoryDuration);
    params.emplace(Config::TRAJ_REF_SPD, m_trajectoryRefSpeed);
    params.emplace(Config::TRAJ_REF_ACC, m_trajectoryRefAccel);
    params.emplace(Config::CMC_PERIOD, m_cmcPeriodMs);
    params.emplace(Config::WAIT_PERIOD, m_waitPeriodMs);
    params.emplace(Config::FRAME, static_cast<double>(referenceFrame));
    params.emplace(Config::STREAMING_CMD, static_cast<double>(streamingCommand));
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

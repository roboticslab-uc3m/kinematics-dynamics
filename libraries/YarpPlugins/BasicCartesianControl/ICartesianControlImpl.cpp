// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath> //-- std::abs

#include <algorithm> // std::transform
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline double getTimestamp(yarp::dev::IPreciselyTimed * iPreciselyTimed)
    {
        return iPreciselyTimed ? iPreciselyTimed->getLastInputStamp().getTime() : yarp::os::Time::now();
    }
}

// ------------------- ICartesianControl Related ------------------------------------

bool BasicCartesianControl::stat(std::vector<double> & x, int * state, double * timestamp)
{
    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return false;
    }

    if (timestamp)
    {
        *timestamp = getTimestamp(iPreciselyTimed);
    }

    if (!iCartesianSolver->fwdKin(currentQ, x))
    {
        yCError(BCC) << "fwdKin() failed";
        return false;
    }

    if (state)
    {
        *state = getCurrentState();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return false;
    }

    if (!iCartesianSolver->invKin(xd, currentQ, q, referenceFrame))
    {
        yCError(BCC) << "invKin() failed";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::movj(const std::vector<double> &xd)
{
    std::vector<double> currentQ(numJoints), qd;

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return false;
    }

    if (!iCartesianSolver->invKin(xd, currentQ, qd, referenceFrame))
    {
        yCError(BCC) << "invKin() failed";
        return false;
    }

    std::vector<double> vmo(numJoints);

    computeIsocronousSpeeds(currentQ, qd, vmo);
    vmoStored.resize(numJoints);

    if (!iPositionControl->getRefSpeeds(vmoStored.data()))
    {
         yCError(BCC) << "getRefSpeeds() (for storing) failed";
         return false;
    }

    if (!iPositionControl->setRefSpeeds(vmo.data()))
    {
         yCError(BCC) << "setRefSpeeds() failed";
         return false;
    }

    //-- Enter position mode and perform movement
    if (!setControlModes(VOCAB_CM_POSITION))
    {
        yCError(BCC) << "Unable to set position mode";
        return false;
    }

    if (!iPositionControl->positionMove(qd.data()))
    {
        yCError(BCC) << "positionMove() failed";
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    cmcSuccess = true;
    yCInfo(BCC) << "Performing MOVJ";

    setCurrentState(VOCAB_CC_MOVJ_CONTROLLING);

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::relj(const std::vector<double> &xd)
{
    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        return movj(xd);
    }

    std::vector<double> x;

    if (!stat(x))
    {
        yCError(BCC) << "stat() failed";
        return false;
    }

    for (unsigned int i = 0; i < xd.size(); i++)
    {
        x[i] += xd[i];
    }

    return movj(x);
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::movl(const std::vector<double> &xd)
{
    yCWarning(BCC) << "MOVL mode still experimental";

    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return false;
    }

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->fwdKin(currentQ, x_base_tcp))
    {
        yCError(BCC) << "fwdKin() failed";
        return false;
    }

    std::vector<double> xd_obj;

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        if (!iCartesianSolver->changeOrigin(xd, x_base_tcp, xd_obj))
        {
            yCError(BCC) << "changeOrigin() failed";
            return false;
        }
    }
    else
    {
        xd_obj = xd;
    }

    trajectories.clear();

    //-- Create line trajectories (one per endpoint if robot is a kin-tree)
    for (unsigned int i = 0; i < xd.size() / 6; i++)
    {
        std::vector<double> xd_base_tcp_sub(x_base_tcp.cbegin() + i * 6, x_base_tcp.cbegin() + (i + 1) * 6);
        std::vector<double> xd_obj_sub(xd_obj.cbegin() + i * 6, xd_obj.cbegin() + (i + 1) * 6);

        auto H_base_start = KdlVectorConverter::vectorToFrame(xd_base_tcp_sub);
        auto H_base_end = KdlVectorConverter::vectorToFrame(xd_obj_sub);

        auto * interpolator = new KDL::RotationalInterpolation_SingleAxis();
        auto * path = new KDL::Path_Line(H_base_start, H_base_end, interpolator, 1.0);
        auto * profile = new KDL::VelocityProfile_Trap(10.0, 10.0);

        trajectories.emplace_back(new KDL::Trajectory_Segment(path, profile, duration));
    }

    if (enableFailFast && !doFailFastChecks(currentQ))
    {
        yCError(BCC) << "Fail-fast checks failed";
        return false;
    }

    //-- Set velocity mode and set state which makes periodic thread implement control.
    if (!setControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Unable to set velocity mode";
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    movementStartTime = yarp::os::Time::now();
    cmcSuccess = true;
    yCInfo(BCC) << "Performing MOVL";

    setCurrentState(VOCAB_CC_MOVL_CONTROLLING);

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::movv(const std::vector<double> &xdotd)
{
    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return false;
    }

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->fwdKin(currentQ, x_base_tcp))
    {
        yCError(BCC) << "fwdKin() failed";
        return false;
    }

    trajectories.clear();

    for (unsigned int i = 0; i < xdotd.size() / 6; i++)
    {
        std::vector<double> xd_base_tcp_sub(x_base_tcp.cbegin() + i * 6, x_base_tcp.cbegin() + (i + 1) * 6);
        std::vector<double> xdotd_sub(xdotd.cbegin() + i * 6, xdotd.cbegin() + (i + 1) * 6);

        auto H_base_start = KdlVectorConverter::vectorToFrame(xd_base_tcp_sub);
        auto twist_in_base = KdlVectorConverter::vectorToTwist(xdotd_sub);

        auto * interpolator = new KDL::RotationalInterpolation_SingleAxis();
        auto * path = new KDL::Path_Line(H_base_start, twist_in_base, interpolator, 1.0);
        auto * profile = new KDL::VelocityProfile_Rectangular(10.0);
        profile->SetProfileDuration(0, 10.0, 10.0 / path->PathLength());

        trajectories.emplace_back(new KDL::Trajectory_Segment(path, profile));
    }

    //-- Set velocity mode and set state which makes periodic thread implement control.
    if (!setControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Unable to set velocity mode";
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    movementStartTime = yarp::os::Time::now();
    cmcSuccess = true;
    yCInfo(BCC) << "Performing MOVV";

    setCurrentState(VOCAB_CC_MOVV_CONTROLLING);

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::gcmp()
{
    //-- Set torque mode and set state which makes periodic thread implement control.
    if (!setControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Unable to set torque mode";
        return false;
    }

    setCurrentState(VOCAB_CC_GCMP_CONTROLLING);
    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::forc(const std::vector<double> &td)
{
    yCWarning(BCC) << "FORC mode still experimental";

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        yCWarning(BCC) << "TCP frame not supported yet in forc command";
        return false;
    }

    //-- Set torque mode and set state which makes periodic thread implement control.
    this->td = td;

    if (!setControlModes(VOCAB_CM_TORQUE))
    {
        yCError(BCC) << "Unable to set torque mode";
        return false;
    }

    setCurrentState(VOCAB_CC_FORC_CONTROLLING);
    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::stopControl()
{
    setCurrentState(VOCAB_CC_NOT_CONTROLLING);

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

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::wait(double timeout)
{
    int state = getCurrentState();

    if (state != VOCAB_CC_MOVJ_CONTROLLING && state != VOCAB_CC_MOVL_CONTROLLING)
    {
        return true;
    }

    double start = yarp::os::Time::now();

    while (state != VOCAB_CC_NOT_CONTROLLING)
    {
        if (timeout != 0.0 && yarp::os::Time::now() - start > timeout)
        {
            yCWarning(BCC, "Timeout reached (%f seconds), stopping control", timeout);
            stopControl();
            break;
        }

        yarp::os::Time::delay(waitPeriodMs / 1000.0);
        state = getCurrentState();
    }

    return cmcSuccess;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::tool(const std::vector<double> &x)
{
    if (!iCartesianSolver->restoreOriginalChain())
    {
        yCError(BCC) << "restoreOriginalChain() failed";
        return false;
    }

    if (!iCartesianSolver->appendLink(x))
    {
        yCError(BCC) << "appendLink() failed";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::act(int command)
{
    yCError(BCC) << "act() not implemented";
    return false;
}

void BasicCartesianControl::twist(const std::vector<double> &xdot)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING || streamingCommand != VOCAB_CC_TWIST
            || !checkControlModes(VOCAB_CM_VELOCITY))
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

    if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot, referenceFrame))
    {
        yCError(BCC) << "diffInvKin() failed";
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
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::pose(const std::vector<double> &x, double interval)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING || streamingCommand != VOCAB_CC_POSE
            || !checkControlModes(VOCAB_CM_VELOCITY))
    {
        yCError(BCC) << "Streaming command not preset";
        return;
    }

    StateWatcher watcher([this] { iVelocityControl->velocityMove(std::vector(numJoints, 0.0).data()); });
    std::vector<double> currentQ(numJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yCError(BCC) << "getEncoders() failed";
        return;
    }

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->fwdKin(currentQ, x_base_tcp))
    {
        yCError(BCC) << "fwdKin() failed";
        return;
    }

    std::vector<double> xd_obj;

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        if (!iCartesianSolver->changeOrigin(x, x_base_tcp, xd_obj))
        {
            yCError(BCC) << "changeOrigin() failed";
            return;
        }
    }
    else
    {
        xd_obj = x;
    }

    std::vector<double> xd;

    if (!iCartesianSolver->poseDiff(xd_obj, x_base_tcp, xd))
    {
        yCError(BCC) << "fwdKinError() failed";
        return;
    }

    std::vector<double> xdot(xd.size());
    const double factor = gain / interval;
    std::transform(xd.begin(), xd.end(), xdot.begin(), [factor](double val) { return val * factor; });

    std::vector<double> qdot;

    if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot, referenceFrame))
    {
        yCError(BCC) << "diffInvKin() failed";
        return;
    }

    if (!checkJointLimits(currentQ, qdot) || !checkJointVelocities(qdot))
    {
        yCError(BCC) << "Joint position or velocity limits exceeded, stopping";
        return;
    }

    watcher.suppress();

    if (!iVelocityControl->velocityMove(qdot.data()))
    {
        yCError(BCC) << "velocityMove() failed";
    }
}

// -----------------------------------------------------------------------------

void BasicCartesianControl::movi(const std::vector<double> &x)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING || streamingCommand != VOCAB_CC_MOVI
            || !checkControlModes(VOCAB_CM_POSITION_DIRECT))
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

    if (!iCartesianSolver->invKin(x, currentQ, q, referenceFrame))
    {
        yCError(BCC) << "invKin() failed";
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
    }
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::setParameter(int vocab, double value)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        yCError(BCC) << "Unable to set config parameter while controlling";
        return false;
    }

    switch (vocab)
    {
    case VOCAB_CC_CONFIG_GAIN:
        if (value < 0.0)
        {
            yCError(BCC) << "Controller gain cannot be negative";
            return false;
        }
        gain = value;
        break;
    case VOCAB_CC_CONFIG_TRAJ_DURATION:
        if (value <= 0.0)
        {
            yCError(BCC) << "Trajectory duration cannot be negative nor zero";
            return false;
        }
        duration = value;
        break;
    case VOCAB_CC_CONFIG_CMC_PERIOD:
        if (!yarp::os::PeriodicThread::setPeriod(value * 0.001))
        {
            yCError(BCC) << "Cannot set new CMC period";
            return false;
        }
        cmcPeriodMs = value;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        if (value <= 0.0)
        {
            yCError(BCC) << "Wait period cannot be negative nor zero";
            return false;
        }
        waitPeriodMs = value;
        break;
    case VOCAB_CC_CONFIG_FRAME:
        if (value != ICartesianSolver::BASE_FRAME && value != ICartesianSolver::TCP_FRAME)
        {
            yCError(BCC) << "Unrecognized or unsupported reference frame vocab";
            return false;
        }
        referenceFrame = static_cast<ICartesianSolver::reference_frame>(value);
        break;
    case VOCAB_CC_CONFIG_STREAMING_CMD:
        if (!presetStreamingCommand(value))
        {
            yCError(BCC) << "Unable to preset streaming command";
            return false;
        }
        streamingCommand = value;
        break;
    default:
        yCError(BCC) << "Unrecognized or unsupported config parameter key:" << yarp::os::Vocab32::decode(vocab);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::getParameter(int vocab, double * value)
{
    switch (vocab)
    {
    case VOCAB_CC_CONFIG_GAIN:
        *value = gain;
        break;
    case VOCAB_CC_CONFIG_TRAJ_DURATION:
        *value = duration;
        break;
    case VOCAB_CC_CONFIG_CMC_PERIOD:
        *value = cmcPeriodMs;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        *value = waitPeriodMs;
        break;
    case VOCAB_CC_CONFIG_FRAME:
        *value = referenceFrame;
        break;
    case VOCAB_CC_CONFIG_STREAMING_CMD:
        *value = streamingCommand;
        break;
    default:
        yCError(BCC) << "Unrecognized or unsupported config parameter key:" << yarp::os::Vocab32::decode(vocab);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::setParameters(const std::map<int, double> & params)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        yCError(BCC) << "Unable to set config parameters while controlling";
        return false;
    }

    bool ok = true;

    for (const auto & [vocab, value] : params)
    {
        ok &= setParameter(vocab, value);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::getParameters(std::map<int, double> & params)
{
    params.emplace(VOCAB_CC_CONFIG_GAIN, gain);
    params.emplace(VOCAB_CC_CONFIG_TRAJ_DURATION, duration);
    params.emplace(VOCAB_CC_CONFIG_CMC_PERIOD, cmcPeriodMs);
    params.emplace(VOCAB_CC_CONFIG_WAIT_PERIOD, waitPeriodMs);
    params.emplace(VOCAB_CC_CONFIG_FRAME, referenceFrame);
    params.emplace(VOCAB_CC_CONFIG_STREAMING_CMD, streamingCommand);
    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath>  //-- std::abs
#include <algorithm>
#include <functional>
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

// -----------------------------------------------------------------------------

namespace
{
    inline double getTimestamp(yarp::dev::IPreciselyTimed * iPreciselyTimed)
    {
        return iPreciselyTimed != NULL ? iPreciselyTimed->getLastInputStamp().getTime() : yarp::os::Time::now();
    }
}

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::BasicCartesianControl::stat(std::vector<double> &x, int * state, double * timestamp)
{
    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "getEncoders() failed";
        return false;
    }

    if (timestamp != NULL)
    {
        *timestamp = getTimestamp(iPreciselyTimed);
    }

    if (!iCartesianSolver->fwdKin(currentQ, x))
    {
        yError() << "fwdKin() failed";
        return false;
    }

    if (state != 0)
    {
        *state = getCurrentState();
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "getEncoders() failed";
        return false;
    }

    if (!iCartesianSolver->invKin(xd, currentQ, q, referenceFrame))
    {
        yError() << "invKin() failed";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::movj(const std::vector<double> &xd)
{
    std::vector<double> currentQ(numRobotJoints), qd;

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "getEncoders() failed";
        return false;
    }

    if (!iCartesianSolver->invKin(xd, currentQ, qd, referenceFrame))
    {
        yError() << "invKin() failed";
        return false;
    }

    std::vector<double> vmo(numRobotJoints);

    computeIsocronousSpeeds(currentQ, qd, vmo);
    vmoStored.resize(numRobotJoints);

    if (!iPositionControl->getRefSpeeds(vmoStored.data()))
    {
         yError() << "getRefSpeeds() (for storing) failed";
         return false;
    }

    if (!iPositionControl->setRefSpeeds(vmo.data()))
    {
         yError() << "setRefSpeeds() failed";
         return false;
    }

    //-- Enter position mode and perform movement
    if (!setControlModes(VOCAB_CM_POSITION))
    {
        yError() << "Unable to set position mode";
        return false;
    }

    if (!iPositionControl->positionMove(qd.data()))
    {
        yError() << "positionMove() failed";
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    cmcSuccess = true;
    yInfo() << "Performing MOVJ";

    setCurrentState(VOCAB_CC_MOVJ_CONTROLLING);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::relj(const std::vector<double> &xd)
{
    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        return movj(xd);
    }

    std::vector<double> x;

    if (!stat(x))
    {
        yError() << "stat() failed";
        return false;
    }

    for (unsigned int i = 0; i < xd.size(); i++)
    {
        x[i] += xd[i];
    }

    return movj(x);
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::movl(const std::vector<double> &xd)
{
    yWarning() << "MOVL mode still experimental";

    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "getEncoders() failed";
        return false;
    }

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->fwdKin(currentQ, x_base_tcp))
    {
        yError() << "fwdKin() failed";
        return false;
    }

    std::vector<double> xd_obj;

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        if (!iCartesianSolver->changeOrigin(xd, x_base_tcp, xd_obj))
        {
            yError() << "changeOrigin() failed";
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

        trajectories.emplace_back(std::make_unique<KDL::Trajectory_Segment>(path, profile, duration));
    }

    //-- Set velocity mode and set state which makes periodic thread implement control.
    if (!setControlModes(VOCAB_CM_VELOCITY))
    {
        yError() << "Unable to set velocity mode";
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    movementStartTime = yarp::os::Time::now();
    cmcSuccess = true;
    yInfo() << "Performing MOVL";

    setCurrentState(VOCAB_CC_MOVL_CONTROLLING);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::movv(const std::vector<double> &xdotd)
{
    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "getEncoders() failed";
        return false;
    }

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->fwdKin(currentQ, x_base_tcp))
    {
        yError() << "fwdKin() failed";
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

        trajectories.emplace_back(std::make_unique<KDL::Trajectory_Segment>(path, profile));
    }

    //-- Set velocity mode and set state which makes periodic thread implement control.
    if (!setControlModes(VOCAB_CM_VELOCITY))
    {
        yError() << "Unable to set velocity mode";
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    movementStartTime = yarp::os::Time::now();
    cmcSuccess = true;
    yInfo() << "Performing MOVV";

    setCurrentState(VOCAB_CC_MOVV_CONTROLLING);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::gcmp()
{
    //-- Set torque mode and set state which makes periodic thread implement control.
    if (!setControlModes(VOCAB_CM_TORQUE))
    {
        yError() << "Unable to set torque mode";
        return false;
    }

    setCurrentState(VOCAB_CC_GCMP_CONTROLLING);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::forc(const std::vector<double> &td)
{
    yWarning() << "FORC mode still experimental";

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        yWarning() << "TCP frame not supported yet in forc command";
        return false;
    }

    //-- Set torque mode and set state which makes periodic thread implement control.
    this->td = td;

    if (!setControlModes(VOCAB_CM_TORQUE))
    {
        yError() << "Unable to set torque mode";
        return false;
    }

    setCurrentState(VOCAB_CC_FORC_CONTROLLING);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::stopControl()
{
    setCurrentState(VOCAB_CC_NOT_CONTROLLING);

    // first switch control so that manipulators don't fall due to e.g. gravity
    if (!setControlModes(VOCAB_CM_POSITION))
    {
        yWarning() << "setControlModes(VOCAB_CM_POSITION) failed";
    }

    // stop joints if already controlling position
    if (!iPositionControl->stop())
    {
        yWarning() << "stop() failed";
    }

    trajectories.clear();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::wait(double timeout)
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
            yWarning("Timeout reached (%f seconds), stopping control", timeout);
            stopControl();
            break;
        }

        yarp::os::Time::delay(waitPeriodMs / 1000.0);
        state = getCurrentState();
    }

    return cmcSuccess;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::tool(const std::vector<double> &x)
{
    if (!iCartesianSolver->restoreOriginalChain())
    {
        yError() << "restoreOriginalChain() failed";
        return false;
    }

    if (!iCartesianSolver->appendLink(x))
    {
        yError() << "appendLink() failed";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::act(int command)
{
    yError() << "act() not implemented";
    return false;
}

void roboticslab::BasicCartesianControl::twist(const std::vector<double> &xdot)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING || streamingCommand != VOCAB_CC_TWIST
            || !checkControlModes(VOCAB_CM_VELOCITY))
    {
        yError() << "Streaming command not preset";
        return;
    }

    std::vector<double> currentQ(numRobotJoints), qdot;

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "getEncoders() failed";
        return;
    }

    if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot, referenceFrame))
    {
        yError() << "diffInvKin() failed";
        return;
    }

    if (!checkJointLimits(currentQ, qdot) || !checkJointVelocities(qdot))
    {
        yError() << "Joint position or velocity limits exceeded, stopping";
        std::fill(qdot.begin(), qdot.end(), 0.0);
        iVelocityControl->velocityMove(qdot.data());
        return;
    }

    if (!iVelocityControl->velocityMove(qdot.data()))
    {
        yError() << "velocityMove() failed";
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::pose(const std::vector<double> &x, double interval)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING || streamingCommand != VOCAB_CC_POSE
            || !checkControlModes(VOCAB_CM_VELOCITY))
    {
        yError() << "Streaming command not preset";
        return;
    }

    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "getEncoders() failed";
        return;
    }

    std::vector<double> x_base_tcp;

    if (!iCartesianSolver->fwdKin(currentQ, x_base_tcp))
    {
        yError() << "fwdKin() failed";
        return;
    }

    std::vector<double> xd_obj;

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        if (!iCartesianSolver->changeOrigin(x, x_base_tcp, xd_obj))
        {
            yError() << "changeOrigin() failed";
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
        yError() << "fwdKinError() failed";
        return;
    }

    std::vector<double> xdot(xd.size());
    const double factor = gain / interval;
    std::transform(xd.begin(), xd.end(), xdot.begin(), std::bind1st(std::multiplies<double>(), factor));

    std::vector<double> qdot;

    if (!iCartesianSolver->diffInvKin(currentQ, xdot, qdot, referenceFrame))
    {
        yError() << "diffInvKin() failed";
        return;
    }

    if (!checkJointLimits(currentQ, qdot) || !checkJointVelocities(qdot))
    {
        yError() << "Joint position or velocity limits exceeded, stopping";
        std::fill(qdot.begin(), qdot.end(), 0.0);
        iVelocityControl->velocityMove(qdot.data());
        return;
    }

    if (!iVelocityControl->velocityMove(qdot.data()))
    {
        yError() << "velocityMove() failed";
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::movi(const std::vector<double> &x)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING || streamingCommand != VOCAB_CC_MOVI
            || !checkControlModes(VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "Streaming command not preset";
        return;
    }

    std::vector<double> currentQ(numRobotJoints), q;

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "getEncoders() failed";
        return;
    }

    if (!iCartesianSolver->invKin(x, currentQ, q, referenceFrame))
    {
        yError() << "invKin() failed";
        return;
    }

    std::vector<double> qdiff(numRobotJoints);

    for (int i = 0; i < numRobotJoints; i++)
    {
        qdiff[i] = q[i] - currentQ[i];
    }

    if (!checkJointLimits(currentQ, qdiff))
    {
        yError() << "Joint position or velocity limits exceeded, not moving";
        return;
    }

    if (!iPositionDirect->setPositions(q.data()))
    {
        yError() << "setPositions() failed";
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::setParameter(int vocab, double value)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        yError() << "Unable to set config parameter while controlling";
        return false;
    }

    switch (vocab)
    {
    case VOCAB_CC_CONFIG_GAIN:
        if (value < 0.0)
        {
            yError() << "Controller gain cannot be negative";
            return false;
        }
        gain = value;
        break;
    case VOCAB_CC_CONFIG_TRAJ_DURATION:
        if (value <= 0.0)
        {
            yError() << "Trajectory duration cannot be negative nor zero";
            return false;
        }
        duration = value;
        break;
    case VOCAB_CC_CONFIG_CMC_PERIOD:
        if (!yarp::os::PeriodicThread::setPeriod(value * 0.001))
        {
            yError() << "Cannot set new CMC period";
            return false;
        }
        cmcPeriodMs = value;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        if (value <= 0.0)
        {
            yError() << "Wait period cannot be negative nor zero";
            return false;
        }
        waitPeriodMs = value;
        break;
    case VOCAB_CC_CONFIG_FRAME:
        if (value != ICartesianSolver::BASE_FRAME && value != ICartesianSolver::TCP_FRAME)
        {
            yError() << "Unrecognized or unsupported reference frame vocab";
            return false;
        }
        referenceFrame = static_cast<ICartesianSolver::reference_frame>(value);
        break;
    case VOCAB_CC_CONFIG_STREAMING_CMD:
        if (!presetStreamingCommand(value))
        {
            yError() << "Unable to preset streaming command";
            return false;
        }
        streamingCommand = value;
        break;
    default:
        yError() << "Unrecognized or unsupported config parameter key:" << yarp::os::Vocab::decode(vocab);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::getParameter(int vocab, double * value)
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
        yError() << "Unrecognized or unsupported config parameter key:" << yarp::os::Vocab::decode(vocab);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::setParameters(const std::map<int, double> & params)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        yError() << "Unable to set config parameters while controlling";
        return false;
    }

    bool ok = true;

    for (std::map<int, double>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
        ok &= setParameter(it->first, it->second);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::getParameters(std::map<int, double> & params)
{
    params.insert(std::make_pair(VOCAB_CC_CONFIG_GAIN, gain));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_TRAJ_DURATION, duration));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_CMC_PERIOD, cmcPeriodMs));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_WAIT_PERIOD, waitPeriodMs));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_FRAME, referenceFrame));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_STREAMING_CMD, streamingCommand));
    return true;
}

// -----------------------------------------------------------------------------

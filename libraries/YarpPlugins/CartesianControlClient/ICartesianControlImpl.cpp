// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- ICartesianControl Related ------------------------------------

void CartesianControlClient::handleStreamingConsumerCmd(Streaming vocab, const std::vector<double> & in)
{
    auto & cmd = commandPort.prepare();

    cmd.clear();
    cmd.addVocab32(static_cast<yarp::conf::vocab32_t>(vocab));

    for (auto i = 0; i < in.size(); i++)
    {
        cmd.addFloat64(in[i]);
    }

    commandPort.write();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::getState(ICartesianControl::ControllerState & state)
{
    if (!fkInPort.isClosed())
    {
        if (!fkStreamResponder.getLastStateData(state, m_fkStreamTimeoutSecs))
        {
            yCWarning(CCC) << "FK stream timeout, falling back to RPC request";
        }
        else
        {
            return yarp::dev::ReturnValue::return_code::return_value_ok;
        }
    }

    auto ret = rpcSender.getState();
    state.x = ret.x;
    state.mode = ret.mode;
    state.timestamp = ret.timestamp;
    state.duration = ret.duration;
    state.progress = ret.progress;
    state.success = ret.success;

    return ret.ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::solvePose(const std::vector<double> & xd, std::vector<double> & q)
{
    auto ret = rpcSender.solvePose(xd);
    q = ret.q;
    return ret.ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::moveJoint(const std::vector<double> & xd)
{
    return rpcSender.moveJoint(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::moveLinear(const std::vector<double> & xd)
{
    return rpcSender.moveLinear(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::moveVelocity(const std::vector<double> & xdotd)
{
    return rpcSender.moveVelocity(xdotd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::gravityCompensation()
{
    return rpcSender.gravityCompensation();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::forceControl(const std::vector<double> & fd)
{
    return rpcSender.forceControl(fd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::stopControl()
{
    return rpcSender.stopControl();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::changeTool(const std::vector<double> & x)
{
    return rpcSender.changeTool(x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::actuateTool(Actuator command)
{
    return rpcSender.actuateTool(command);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::pose(const std::vector<double> & x)
{
    return handleStreamingConsumerCmd(Streaming::POSE, x);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::twist(const std::vector<double> & xdot)
{
    return handleStreamingConsumerCmd(Streaming::TWIST, xdot);
}

// -----------------------------------------------------------------------------

void CartesianControlClient::wrench(const std::vector<double> & w)
{
    return handleStreamingConsumerCmd(Streaming::WRENCH, w);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::setParameter(Config vocab, config_value_t value)
{
    switch (vocab)
    {
        case Config::GAIN:
        case Config::TRAJ_DURATION:
        case Config::TRAJ_REF_SPD:
        case Config::TRAJ_REF_ACC:
        case Config::CMC_PERIOD:
            if (!std::holds_alternative<double>(value))
            {
                yCError(CCC) << "setParameter: expected double value for vocab"
                             << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
                return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
            }

            return rpcSender.setParameterDouble(vocab, std::get<double>(value));
        case Config::FRAME:
        case Config::STREAMING_CMD:
            if (!std::holds_alternative<yarp::conf::vocab32_t>(value))
            {
                yCError(CCC) << "setParameter: expected vocab32_t value for vocab"
                             << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
                return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
            }

            return rpcSender.setParameterVocab(vocab, std::get<yarp::conf::vocab32_t>(value));
        default:
            yCError(CCC) << "setParameter: unknown vocab"
                         << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::getParameter(Config vocab, config_value_t * value)
{
    switch (vocab)
    {
        case Config::GAIN:
        case Config::TRAJ_DURATION:
        case Config::TRAJ_REF_SPD:
        case Config::TRAJ_REF_ACC:
        case Config::CMC_PERIOD:
        {
            auto ret = rpcSender.getParameterDouble(vocab);
            *value = ret.value;
            return ret.ret;
        }
        case Config::FRAME:
        case Config::STREAMING_CMD:
        {
            auto ret = rpcSender.getParameterVocab(vocab);
            *value = ret.value;
            return ret.ret;
        }
        default:
            yCError(CCC) << "getParameter: unknown vocab"
                         << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::setParameters(const config_map_t & params)
{
    std::map<roboticslab::ICartesianControl::Config, double> doubleParams;

    for (const auto & [vocab, value] : params)
    {
        if (std::holds_alternative<double>(value))
        {
            doubleParams[vocab] = std::get<double>(value);
        }
        else if (std::holds_alternative<yarp::conf::vocab32_t>(value))
        {
            doubleParams[vocab] = static_cast<double>(std::get<yarp::conf::vocab32_t>(value));
        }
        else
        {
            yCError(CCC) << "setParameters: unknown value type for vocab"
                         << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }
    }

    return rpcSender.setParameters(doubleParams);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CartesianControlClient::getParameters(config_map_t & params)
{
    auto ret = rpcSender.getParameters();

    for (const auto & [vocab, value] : ret.params)
    {
        switch (vocab)
        {
            case Config::GAIN:
            case Config::TRAJ_DURATION:
            case Config::TRAJ_REF_SPD:
            case Config::TRAJ_REF_ACC:
            case Config::CMC_PERIOD:
                params[vocab] = value;
                break;
            case Config::FRAME:
            case Config::STREAMING_CMD:
                params[vocab] = static_cast<yarp::conf::vocab32_t>(value);
                break;
            default:
                yCError(CCC) << "getParameters: unknown vocab"
                             << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
                return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }
    }

    return ret.ret;
}

// -----------------------------------------------------------------------------

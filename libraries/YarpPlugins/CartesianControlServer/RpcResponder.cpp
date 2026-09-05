// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- RpcResponder Related ------------------------------------

return_get_state RpcResponder::getState()
{
    ICartesianControl::ControllerState state;
    return_get_state ret;
    ret.ret = iCartesianControl->getState(state);
    ret.x = state.x;
    ret.mode = state.mode;
    ret.timestamp = state.timestamp;
    ret.duration = state.duration;
    ret.progress = state.progress;
    ret.success = state.success;
    return ret;
}

// -----------------------------------------------------------------------------

return_solve_pose RpcResponder::solvePose(const std::vector<double> & xd)
{
    return_solve_pose ret;
    ret.ret = iCartesianControl->solvePose(xd, ret.q);
    return ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::moveJoint(const std::vector<double> & xd)
{
    return iCartesianControl->moveJoint(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::moveLinear(const std::vector<double> & xd)
{
    return iCartesianControl->moveLinear(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::moveVelocity(const std::vector<double> & xdotd)
{
    return iCartesianControl->moveVelocity(xdotd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::gravityCompensation()
{
    return iCartesianControl->gravityCompensation();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::forceControl(const std::vector<double> & fd)
{
    return iCartesianControl->forceControl(fd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::stopControl()
{
    return iCartesianControl->stopControl();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::changeTool(const std::vector<double> & x)
{
    return iCartesianControl->changeTool(x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::actuateTool(roboticslab::ICartesianControl::Actuator command)
{
    return iCartesianControl->actuateTool(command);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::setParameterDouble(roboticslab::ICartesianControl::Config vocab, double value)
{
    try
    {
        return iCartesianControl->setParameter(vocab, value);
    }
    catch (const std::bad_variant_access & e)
    {
        yCError(CCS) << e.what();
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::setParameterVocab(roboticslab::ICartesianControl::Config vocab, std::int32_t value)
{
    try
    {
        return iCartesianControl->setParameter(vocab, value);
    }
    catch (const std::bad_variant_access & e)
    {
        yCError(CCS) << e.what();
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
}

// -----------------------------------------------------------------------------

return_get_parameter_double RpcResponder::getParameterDouble(roboticslab::ICartesianControl::Config vocab)
{
    return_get_parameter_double ret;
    ICartesianControl::config_value_t value;
    ret.ret = iCartesianControl->getParameter(vocab, &value);

    try
    {
        ret.value = std::get<double>(value);
    }
    catch (const std::bad_variant_access & e)
    {
        yCError(CCS) << e.what();
        ret.ret = yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return ret;
}

// -----------------------------------------------------------------------------

return_get_parameter_vocab RpcResponder::getParameterVocab(roboticslab::ICartesianControl::Config vocab)
{
    return_get_parameter_vocab ret;
    ICartesianControl::config_value_t value;
    ret.ret = iCartesianControl->getParameter(vocab, &value);

    try
    {
        ret.value = std::get<yarp::conf::vocab32_t>(value);
    }
    catch (const std::bad_variant_access & e)
    {
        yCError(CCS) << e.what();
        ret.ret = yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::setParameters(const std::map<roboticslab::ICartesianControl::Config, double> & params)
{
    ICartesianControl::config_map_t configMap;

    for (const auto & [vocab, value] : params)
    {
        configMap[vocab] = value;
    }

    try
    {
        return iCartesianControl->setParameters(configMap);
    }
    catch (const std::bad_variant_access & e)
    {
        yCError(CCS) << e.what();
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
}

// -----------------------------------------------------------------------------

return_get_parameters RpcResponder::getParameters()
{
    return_get_parameters ret;
    ICartesianControl::config_map_t configMap;
    ret.ret = iCartesianControl->getParameters(configMap);

    for (const auto & [vocab, value] : configMap)
    {
        if (std::holds_alternative<double>(value))
        {
            ret.params[vocab] = std::get<double>(value);
        }
        else if (std::holds_alternative<yarp::conf::vocab32_t>(value))
        {
            ret.params[vocab] = static_cast<double>(std::get<yarp::conf::vocab32_t>(value));
        }
        else
        {
            yCError(CCS) << "getParameters: unexpected value type for vocab"
                         << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
            ret.ret = yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }
    }

    return ret;
}

// -----------------------------------------------------------------------------

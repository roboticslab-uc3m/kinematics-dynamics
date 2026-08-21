// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

using namespace roboticslab;

// ------------------- RpcResponder Related ------------------------------------

return_get_state RpcResponder::getState()
{
    return_get_state ret;
    ret.ret = iCartesianControl->getState(ret.x, &ret.state, &ret.timestamp);
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

yarp::dev::ReturnValue RpcResponder::wait(double timeout)
{
    return iCartesianControl->wait(timeout);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::changeTool(const std::vector<double> & x)
{
    return iCartesianControl->changeTool(x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::actuateTool(const roboticslab::ICartesianControl::Actuator command)
{
    return iCartesianControl->actuateTool(command);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::setParameter(const roboticslab::ICartesianControl::Config vocab, const double value)
{
    return iCartesianControl->setParameter(vocab, value);
}

// -----------------------------------------------------------------------------

return_get_parameter RpcResponder::getParameter(const roboticslab::ICartesianControl::Config vocab)
{
    return_get_parameter ret;
    ret.ret = iCartesianControl->getParameter(vocab, &ret.value);
    return ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::setParameters(const std::map<roboticslab::ICartesianControl::Config, double> & params)
{
    return iCartesianControl->setParameters(params);
}

// -----------------------------------------------------------------------------

return_get_parameters RpcResponder::getParameters()
{
    return_get_parameters ret;
    ret.ret = iCartesianControl->getParameters(ret.params);
    return ret;
}

// -----------------------------------------------------------------------------

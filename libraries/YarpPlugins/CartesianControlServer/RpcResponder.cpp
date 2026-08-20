// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

using namespace roboticslab;

// ------------------- RpcResponder Related ------------------------------------

return_stat RpcResponder::stat()
{
    return_stat ret;
    ret.ret = iCartesianControl->stat(ret.x, &ret.state, &ret.timestamp);
    return ret;
}

// -----------------------------------------------------------------------------

return_inv RpcResponder::inv(const std::vector<double> & xd)
{
    return_inv ret;
    ret.ret = iCartesianControl->inv(xd, ret.q);
    return ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::movj(const std::vector<double> & xd)
{
    return iCartesianControl->movj(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::relj(const std::vector<double> & xd)
{
    return iCartesianControl->relj(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::movl(const std::vector<double> & xd)
{
    return iCartesianControl->movl(xd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::movv(const std::vector<double> & xdotd)
{
    return iCartesianControl->movv(xdotd);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::gcmp()
{
    return iCartesianControl->gcmp();
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::forc(const std::vector<double> & fd)
{
    return iCartesianControl->forc(fd);
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

yarp::dev::ReturnValue RpcResponder::tool(const std::vector<double> & x)
{
    return iCartesianControl->tool(x);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcResponder::act(const roboticslab::ICartesianControl::Actuator command)
{
    return iCartesianControl->act(command);
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

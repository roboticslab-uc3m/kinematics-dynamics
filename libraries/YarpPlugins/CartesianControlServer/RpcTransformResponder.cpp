// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- RpcTransformResponder Related ------------------------------------

return_stat RpcTransformResponder::stat()
{
    return_stat ret = RpcResponder::stat();

    if (ret.ret && !transformOutgoingData(ret.x, ret.x))
    {
        ret.ret = yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return ret;
}

// -----------------------------------------------------------------------------

return_inv RpcTransformResponder::inv(const std::vector<double> & xd)
{
    std::vector<double> transformed;

    if (!transformIncomingData(xd, transformed))
    {
        return_inv ret;
        ret.ret = yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        return ret;
    }

    return RpcResponder::inv(transformed);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcTransformResponder::movj(const std::vector<double> & xd)
{
    std::vector<double> transformed;

    if (!transformIncomingData(xd, transformed))
    {
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return RpcResponder::movj(transformed);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcTransformResponder::relj(const std::vector<double> & xd)
{
    std::vector<double> transformed;

    if (!transformIncomingData(xd, transformed))
    {
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return RpcResponder::relj(transformed);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcTransformResponder::movl(const std::vector<double> & xd)
{
    std::vector<double> transformed;

    if (!transformIncomingData(xd, transformed))
    {
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return RpcResponder::movl(transformed);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcTransformResponder::tool(const std::vector<double> & x)
{
    std::vector<double> transformed;

    if (!transformIncomingData(x, transformed))
    {
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    return RpcResponder::tool(transformed);
}

// -----------------------------------------------------------------------------

bool RpcTransformResponder::transformIncomingData(const std::vector<double> & vin, std::vector<double> & vout)
{
    return KinRepresentation::encodePose(vin, vout, coord, orient, units);
}

// -----------------------------------------------------------------------------

bool RpcTransformResponder::transformOutgoingData(const std::vector<double> & vin, std::vector<double> & vout)
{
    return KinRepresentation::decodePose(vin, vout, coord, orient, units);
}

// -----------------------------------------------------------------------------

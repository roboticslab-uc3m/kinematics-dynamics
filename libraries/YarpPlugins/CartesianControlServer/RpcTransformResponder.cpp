// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- RpcTransformResponder Related ------------------------------------

return_get_state RpcTransformResponder::getState()
{
    return_get_state ret = RpcResponder::getState();

    if (ret.ret && !transformOutgoingData(ret.x, ret.x))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ret.ret = yarp::dev::ReturnValue_error_method_failed;
#else
        ret.ret = yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return ret;
}

// -----------------------------------------------------------------------------

return_solve_pose RpcTransformResponder::solvePose(const std::vector<double> & xd)
{
    std::vector<double> transformed;

    if (!transformIncomingData(xd, transformed))
    {
        return_solve_pose ret;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ret.ret = yarp::dev::ReturnValue_error_method_failed;
#else
        ret.ret = yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        return ret;
    }

    return RpcResponder::solvePose(transformed);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcTransformResponder::moveJoint(const std::vector<double> & xd)
{
    std::vector<double> transformed;

    if (!transformIncomingData(xd, transformed))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return RpcResponder::moveJoint(transformed);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcTransformResponder::moveLinear(const std::vector<double> & xd)
{
    std::vector<double> transformed;

    if (!transformIncomingData(xd, transformed))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return RpcResponder::moveLinear(transformed);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue RpcTransformResponder::changeTool(const std::vector<double> & x)
{
    std::vector<double> transformed;

    if (!transformIncomingData(x, transformed))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return RpcResponder::changeTool(transformed);
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

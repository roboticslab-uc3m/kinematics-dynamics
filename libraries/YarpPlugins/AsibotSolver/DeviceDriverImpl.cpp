// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

#include <yarp/conf/version.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>

#include <yarp/math/Math.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto NUM_MOTORS = 5;
constexpr auto DEFAULT_A0 = 0.3;
constexpr auto DEFAULT_A1 = 0.4;
constexpr auto DEFAULT_A2 = 0.4;
constexpr auto DEFAULT_A3 = 0.3;
constexpr auto DEFAULT_STRATEGY = "leastOverallAngularDisplacement";

// ------------------- DeviceDriver Related ------------------------------------

bool AsibotSolver::open(yarp::os::Searchable& config)
{
#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(ASIBOT) << "Config:" << config.toString();
#endif

    A0 = config.check("A0", yarp::os::Value(DEFAULT_A0), "length of link 1 (meters)").asFloat64();
    A1 = config.check("A1", yarp::os::Value(DEFAULT_A1), "length of link 2 (meters)").asFloat64();
    A2 = config.check("A2", yarp::os::Value(DEFAULT_A2), "length of link 3 (meters)").asFloat64();
    A3 = config.check("A3", yarp::os::Value(DEFAULT_A3), "length of link 4 (meters)").asFloat64();

    yCInfo(ASIBOT, "Using A0: %f, A1: %f, A2: %f, A3: %f", A0, A1, A2, A3);

    if (!config.check("mins") || !config.check("maxs"))
    {
        yCError(ASIBOT) << "Missing 'mins' and/or 'maxs' option(s)";
        return false;
    }

    auto * mins = config.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();
    auto * maxs = config.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();

    if (!mins || !maxs)
    {
        yCError(ASIBOT) << "Empty 'mins' and/or 'maxs' option(s)";
        return false;
    }

    if (mins->size() != NUM_MOTORS || maxs->size() != NUM_MOTORS)
    {
        yCError(ASIBOT, "mins.size(), maxs.size() (%zu, %zu) != NUM_MOTORS (%d)", mins->size(), maxs->size(), NUM_MOTORS);
        return false;
    }

    qMin.resize(NUM_MOTORS);
    qMax.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        qMin[i] = mins->get(i).asFloat64();
        qMax[i] = maxs->get(i).asFloat64();

        if (qMin[i] == qMax[i])
        {
            yCWarning(ASIBOT, "qMin[%1$d] == qMax[%1$d] (%2$f)", i, qMin[i]);
        }
        if (qMin[i] > qMax[i])
        {
            yCError(ASIBOT, "qMin[%1$d] > qMax[%1$d] (%2$f > %3$f)", i, qMin[i], qMax[i]);
            return false;
        }
    }

    auto strategy = config.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

    if (strategy == DEFAULT_STRATEGY)
    {
        confFactory = new AsibotConfigurationLeastOverallAngularDisplacementFactory(qMin, qMax);
    }
    else
    {
        yCError(ASIBOT) << "Unsupported IK configuration strategy:" << strategy;
        return false;
    }

    tcpFrameStruct.hasFrame = false;
    tcpFrameStruct.frameTcp = yarp::math::eye(4);

    return true;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::close() {
    if (confFactory)
    {
        delete confFactory;
        confFactory = nullptr;
    }

    return true;
}

// -----------------------------------------------------------------------------

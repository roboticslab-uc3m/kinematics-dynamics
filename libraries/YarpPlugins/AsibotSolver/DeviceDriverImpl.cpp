// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include <yarp/math/Math.h>

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AsibotSolver::open(yarp::os::Searchable& config)
{
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    A0 = config.check("A0", yarp::os::Value(DEFAULT_A0), "length of link 1 (meters)").asDouble();
    A1 = config.check("A1", yarp::os::Value(DEFAULT_A1), "length of link 2 (meters)").asDouble();
    A2 = config.check("A2", yarp::os::Value(DEFAULT_A2), "length of link 3 (meters)").asDouble();
    A3 = config.check("A3", yarp::os::Value(DEFAULT_A3), "length of link 4 (meters)").asDouble();

    CD_INFO("AsibotSolver using A0: %f, A1: %f, A2: %f, A3: %f.\n", A0, A1, A2, A3);

    if (!config.check("mins") || !config.check("maxs"))
    {
        CD_ERROR("Missing 'mins' and/or 'maxs' option(s).\n");
        return false;
    }

    yarp::os::Bottle *mins = config.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();
    yarp::os::Bottle *maxs = config.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();

    if (mins == YARP_NULLPTR || maxs == YARP_NULLPTR)
    {
        CD_ERROR("Empty 'mins' and/or 'maxs' option(s)\n");
        return false;
    }

    if (mins->size() != NUM_MOTORS || maxs->size() != NUM_MOTORS)
    {
        CD_ERROR("mins.size(), maxs.size() (%d, %d) != NUM_MOTORS (%d)\n", mins->size(), maxs->size(), NUM_MOTORS);
        return false;
    }

    qMin.resize(NUM_MOTORS);
    qMax.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        qMin[i] = mins->get(i).asDouble();
        qMax[i] = maxs->get(i).asDouble();

        if (qMin[i] == qMax[i])
        {
            CD_WARNING("qMin[%1$d] == qMax[%1$d] (%2$f)\n", i, qMin[i]);
        }
        if (qMin[i] > qMax[i])
        {
            CD_ERROR("qMin[%1$d] > qMax[%1$d] (%2$f > %3$f)\n", i, qMin[i], qMax[i]);
            return false;
        }
    }

    std::string strategy = config.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

    if (!buildStrategyFactory(strategy))
    {
        CD_ERROR("Unsupported IK configuration strategy: %s.\n", strategy.c_str());
        return false;
    }

    tcpFrameStruct.hasFrame = false;
    tcpFrameStruct.frameTcp = yarp::math::eye(4);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::close() {
    if (confFactory != NULL)
    {
        delete confFactory;
        confFactory = NULL;
    }

    return true;
}

// -----------------------------------------------------------------------------

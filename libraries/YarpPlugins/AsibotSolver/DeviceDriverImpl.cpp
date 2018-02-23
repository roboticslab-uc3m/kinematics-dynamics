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

#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AsibotSolver::open(yarp::os::Searchable& config)
{
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    std::printf("--------------------------------------------------------------\n");

    if (config.check("help"))
    {
        std::printf("AsibotSolver options:\n");
        std::printf("\t--help (this help)\n");
        std::printf("\t--A0 [m] (dist from base to motor 2, default: \"%f\")\n", DEFAULT_A0);
        std::printf("\t--A1 [m] (dist from motor 2 to motor 3, default: \"%f\")\n", DEFAULT_A1);
        std::printf("\t--A2 [m] (dist from motor 3 to motor 4, default: \"%f\")\n", DEFAULT_A2);
        std::printf("\t--A3 [m] (dist from motor 4 to end-effector, default: \"%f\")\n", DEFAULT_A3);
        std::printf("\t--invKinStrategy (IK configuration strategy, default: \"%s\")\n", DEFAULT_STRATEGY);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    A0 = config.check("A0", yarp::os::Value(DEFAULT_A0), "length of link 1").asDouble();
    A1 = config.check("A1", yarp::os::Value(DEFAULT_A1), "length of link 2").asDouble();
    A2 = config.check("A2", yarp::os::Value(DEFAULT_A2), "length of link 3").asDouble();
    A3 = config.check("A3", yarp::os::Value(DEFAULT_A3), "length of link 4").asDouble();

    CD_INFO("AsibotSolver using A0: %f, A1: %f, A2: %f, A3: %f.\n", A0, A1, A2, A3);

    if (!config.check("mins") || !config.check("maxs"))
    {
        CD_ERROR("Missing 'mins' and/or 'maxs' option(s).\n");
        return false;
    }

    yarp::os::Bottle *mins = config.findGroup("mins", "joint lower limits").get(1).asList();
    yarp::os::Bottle *maxs = config.findGroup("maxs", "joint upper limits").get(1).asList();

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

        if (qMin[i] > qMax[i])
        {
            CD_ERROR("qMin > qMax (%f > %f) at joint %d\n", qMin[i], qMax[i], i);
            return false;
        }
    }

    std::string strategy = config.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

    if (!buildStrategyFactory(strategy))
    {
        CD_ERROR("Unsupported IK configuration strategy: %s.\n", strategy.c_str());
        return false;
    }

    if (config.check("help"))
    {
        std::exit(0);
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

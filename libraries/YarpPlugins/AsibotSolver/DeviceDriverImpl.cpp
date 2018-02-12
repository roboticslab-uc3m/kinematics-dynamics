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

    yarp::os::Value mins = config.check("mins", yarp::os::Value::getNullValue(), "minimum joint limits");

    if (!mins.isNull())
    {
        yarp::os::Bottle * b = mins.asList();

        for (int i = 0; i < b->size(); i++)
        {
            qMin.push_back(b->get(i).asDouble());
        }
    }
    else
    {
        qMin.resize(NUM_MOTORS, -90);
    }

    yarp::os::Value maxs = config.check("maxs", yarp::os::Value::getNullValue(), "maximum joint limits");

    if (!maxs.isNull())
    {
        yarp::os::Bottle * b = maxs.asList();

        for (int i = 0; i < b->size(); i++)
        {
            qMax.push_back(b->get(i).asDouble());
        }
    }
    else
    {
        qMax.resize(NUM_MOTORS, 90);
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

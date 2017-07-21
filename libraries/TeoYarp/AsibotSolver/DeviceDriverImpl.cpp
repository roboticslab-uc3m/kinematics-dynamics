// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <AsibotSolver.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AsibotSolver::open(yarp::os::Searchable& config)
{
    startTime = 0;
    realRad.resize(5);
    targetX.resize(3);
    targetO.resize(2);

    A0 = DEFAULT_A0;
    A1 = DEFAULT_A1;
    A2 = DEFAULT_A2;
    A3 = DEFAULT_A3;
    tool = DEFAULT_TOOL;

    printf("--------------------------------------------------------------\n");
    if(config.check("help")) {
        printf("CartesianBot options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--A0 [m] (dist from base to motor 2, default: \"%f\")\n",A0);
        printf("\t--A1 [m] (dist from motor 2 to motor 3, default: \"%f\")\n",A1);
        printf("\t--A2 [m] (dist from motor 3 to motor 4, default: \"%f\")\n",A2);
        printf("\t--A3 [m] (dist from motor 4 to end-effector, default: \"%f\")\n",A3);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (config.check("A0")) A0 = config.find("A0").asDouble();
    if (config.check("A1")) A1 = config.find("A1").asDouble();
    if (config.check("A2")) A2 = config.find("A2").asDouble();
    if (config.check("A3")) A3 = config.find("A3").asDouble();

    CD_DEBUG("CartesianBot using A0: %f, A1: %f, A2: %f, A3: %f.\n", A0, A1, A2, A3);

    qMin.resize(NUM_MOTORS, -90);
    qMax.resize(NUM_MOTORS, 90);

    conf = new AsibotConfigurationLeastOverallAngularDisplacement(qMin, qMax);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::close() {
    if (conf != NULL)
    {
        delete conf;
        conf = NULL;
    }

    return true;
}

// -----------------------------------------------------------------------------

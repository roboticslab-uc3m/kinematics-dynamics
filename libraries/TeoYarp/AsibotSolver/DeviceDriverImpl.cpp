// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <AsibotSolver.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AsibotSolver::open(yarp::os::Searchable& config)
{
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    std::string kinematics = config.check("kinematics", yarp::os::Value(DEFAULT_KINEMATICS), "limb kinematic description").asString();
    CD_INFO("kinematics: %s [%s]\n", kinematics.c_str(), DEFAULT_KINEMATICS);

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("kinematics");

    std::string kinematicsFullPath = rf.findFileByName(kinematics);

    yarp::os::Property fullConfig;
    fullConfig.fromConfigFile(kinematicsFullPath.c_str());
    fullConfig.fromString(config.toString(), false);

    CD_DEBUG("fullConfig: %s.\n", fullConfig.toString().c_str());

    startTime = 0;
    realRad.resize(5);
    targetX.resize(3);
    targetO.resize(2);

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

    A0 = fullConfig.check("A0", yarp::os::Value(DEFAULT_A0), "length of link 1").asDouble();
    A1 = fullConfig.check("A1", yarp::os::Value(DEFAULT_A1), "length of link 2").asDouble();
    A2 = fullConfig.check("A2", yarp::os::Value(DEFAULT_A2), "length of link 3").asDouble();
    A3 = fullConfig.check("A3", yarp::os::Value(DEFAULT_A3), "length of link 4").asDouble();

    tool = DEFAULT_TOOL;

    CD_DEBUG("CartesianBot using A0: %f, A1: %f, A2: %f, A3: %f.\n", A0, A1, A2, A3);

    yarp::os::Value min = fullConfig.check("qMin", yarp::os::Value::getNullValue(), "minimum joint limits");

    if (!min.isNull())
    {
        yarp::os::Bottle * b = min.asList();

        for (int i = 0; i < b->size(); i++)
        {
            qMin.push_back(b->get(i).asDouble());
        }
    }
    else
    {
        qMin.resize(NUM_MOTORS, -90);
    }

    yarp::os::Value max = fullConfig.check("qMax", yarp::os::Value::getNullValue(), "maximum joint limits");

    if (!max.isNull())
    {
        yarp::os::Bottle * b = max.asList();

        for (int i = 0; i < b->size(); i++)
        {
            qMax.push_back(b->get(i).asDouble());
        }
    }
    else
    {
        qMax.resize(NUM_MOTORS, 90);
    }

    std::string strategy = fullConfig.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

    if (strategy == DEFAULT_STRATEGY)
    {
        conf = new AsibotConfigurationLeastOverallAngularDisplacement(qMin, qMax);
    }
    else
    {
        CD_ERROR("Unsupported IK configuration strategy: %s.\n", strategy.c_str());
        return false;
    }

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

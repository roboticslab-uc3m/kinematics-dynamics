// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianBot.h"

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianBot::open(Searchable& config) {

    isQuiet = false;

    cmc_status = 0;
    startTime = 0;
    realRad.resize(5);
    targetX.resize(3);
    targetO.resize(2);

    A0 = DEFAULT_A0;
    A1 = DEFAULT_A1;
    A2 = DEFAULT_A2;
    A3 = DEFAULT_A3;
    cmcMs = DEFAULT_CMC_MS;
    duration = DEFAULT_DURATION;
    maxAcc = DEFAULT_MAXACC;
    maxVel = DEFAULT_MAXVEL;
    ConstString strRobotDevice = DEFAULT_ROBOT_DEVICE;
    ConstString strRobotSubDevice = DEFAULT_ROBOT_SUBDEVICE;
    ConstString strRobotName = DEFAULT_ROBOT_NAME;
    ConstString strRobotLocal = DEFAULT_ROBOT_LOCAL;
    ConstString strRobotRemote = DEFAULT_ROBOT_REMOTE;
    tool = DEFAULT_TOOL;

    printf("--------------------------------------------------------------\n");
    if(config.check("help")) {
        printf("CartesianBot options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--A0 [m] (dist from base to motor 2, default: \"%f\")\n",A0);
        printf("\t--A1 [m] (dist from motor 2 to motor 3, default: \"%f\")\n",A1);
        printf("\t--A2 [m] (dist from motor 3 to motor 4, default: \"%f\")\n",A2);
        printf("\t--A3 [m] (dist from motor 4 to end-effector, default: \"%f\")\n",A3);
        printf("\t--cmcMs [ms] (rate of Cartesian Motion Controller thread, default: \"%f\")\n",cmcMs);
        printf("\t--duration [s] (duration of movl movements, default: \"%f\")\n",duration);
        printf("\t--maxAcc [units/s^2] (maximum joint acceleration, default: \"%f\")\n",maxAcc);
        printf("\t--maxVel [units/s] (maximum joint velocity, default: \"%f\")\n",maxVel);
        printf("\t--robotDevice (device we create, default: \"%s\")\n",strRobotDevice.c_str());
        printf("\t--robotSubdevice (library we use, default: \"%s\")\n",strRobotSubDevice.c_str());
        printf("\t--robotName (port name of created device, default: \"%s\")\n",strRobotName.c_str());
        printf("\t--robotLocal (if accesing remote, local port name, default: \"%s\")\n",strRobotLocal.c_str());
        printf("\t--robotRemote (if accesing remote, remote port name, default: \"%s\")\n",strRobotRemote.c_str());
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (config.check("A0")) A0 = config.find("A0").asDouble();
    if (config.check("A1")) A1 = config.find("A1").asDouble();
    if (config.check("A2")) A2 = config.find("A2").asDouble();
    if (config.check("A3")) A3 = config.find("A3").asDouble();
    printf("CartesianBot using A0: %f, A1: %f, A2: %f, A3: %f.\n",A0,A1,A2,A3);

    if (config.check("cmcMs")) cmcMs = config.find("cmcMs").asDouble();
    if (config.check("duration")) duration = config.find("duration").asDouble();
    if (config.check("maxAcc")) maxAcc = config.find("maxAcc").asDouble();
    if (config.check("maxVel")) maxVel = config.find("maxVel").asDouble();
    printf("CartesianBot using cmcMs: %f, duration: %f, maxAcc: %f, maxVel: %f.\n",cmcMs,duration,maxAcc,maxVel);

    if (config.check("robotDevice")) strRobotDevice = config.find("robotDevice").asString();
    if (config.check("robotSubDevice")) strRobotSubDevice = config.find("robotSubDevice").asString();
    if (config.check("robotName")) strRobotName = config.find("robotName").asString();
    if (config.check("robotLocal")) strRobotLocal = config.find("robotLocal").asString();
    if (config.check("robotRemote")) strRobotRemote = config.find("robotRemote").asString();
    printf("CartesianBot using robotDevice: %s, robotSubDevice: %s, robotName: %s.\n",strRobotDevice.c_str(),strRobotSubDevice.c_str(),strRobotName.c_str());
    printf("CartesianBot using robotLocal: %s, robotRemote: %s.\n",strRobotLocal.c_str(),strRobotRemote.c_str());

    if( (strRobotDevice=="remote_controlboard") && (config.check("help")) ) {
        printf("--------------------------------------------------------------\n");
        ::exit(1);
    }

    Property options;
    options.fromString(config.toString());
    options.put("device",strRobotDevice);
    options.put("subdevice",strRobotSubDevice);
    options.put("name",strRobotName);
    options.put("local",strRobotLocal);
    options.put("remote",strRobotRemote);

    robotDevice.open(options);
    if (!robotDevice.isValid()) {
        //%printf("Robot device not available. Here are the known devices:\n");
        //printf("%s", Drivers::factory().toString().c_str());
        fprintf(stderr, "[CartesianBot] warning: Robot device not available.\n");
        return false;
    }

    bool ok = true;
    ok &= robotDevice.view(vel);
    ok &= robotDevice.view(enc);
    ok &= robotDevice.view(mode);
    if (!ok) {
        fprintf(stderr, "[CartesianBot] error: Problems acquiring robot interfaces.\n");
        return false;
    } else printf("[CartesianBot] success: Acquired robot interfaces.\n");

    // Start the RateThread
    this->setRate(cmcMs);
    this->start();
    return true;
}

// -----------------------------------------------------------------------------

bool CartesianBot::close() {
    // printf("Cleaned heap.\n");
    return true;
}

// -----------------------------------------------------------------------------


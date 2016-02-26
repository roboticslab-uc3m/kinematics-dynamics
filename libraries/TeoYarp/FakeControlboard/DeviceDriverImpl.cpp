// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::FakeControlboard::open(yarp::os::Searchable& config) {

    axes = config.check("axes",DEFAULT_AXES,"number of axes to control").asInt();
    double genInitPos = config.check("genInitPos",DEFAULT_GEN_INIT_POS,"general initialization positions").asDouble();
    double genJointTol = config.check("genJointTol",DEFAULT_GEN_JOINT_TOL,"general joint tolerances").asDouble();
    double genMaxLimit = config.check("genMaxLimit",DEFAULT_GEN_MAX_LIMIT,"general max limits").asDouble();
    double genMinLimit = config.check("genMinLimit",DEFAULT_GEN_MIN_LIMIT,"general min limits").asDouble();
    double genRefSpeed = config.check("genRefSpeed",DEFAULT_GEN_REF_SPEED,"general ref speed").asDouble();
    double genEncRawExposed = config.check("genEncRawExposed",DEFAULT_GEN_ENC_RAW_EXPOSED,"general EncRawExposed").asDouble();
    double genVelRawExposed = config.check("genVelRawExposed",DEFAULT_GEN_VEL_RAW_EXPOSED,"general VelRawExposed").asDouble();
    modePosVel = config.check("modePosVel",DEFAULT_MODE_POS_VEL,"0:pos, 1:vel").asInt();

    yarp::os::Bottle* initPoss;
    if (config.check("initPoss")) {
        initPoss = config.find("initPoss").asList();
        printf("FakeControlboard using individual initPoss: %s\n",initPoss->toString().c_str());
        if(initPoss->size() != axes) printf("[warning] initPoss->size() != axes\n");
    } else {
        initPoss = 0;
        printf("FakeControlboard not using individual initPoss, defaulting to genInitPos.\n");
    }
    yarp::os::Bottle* jointTols;
    if (config.check("jointTols")) {
        jointTols = config.find("jointTols").asList();
        printf("FakeControlboard using individual jointTols: %s\n",jointTols->toString().c_str());
        if(jointTols->size() != axes) printf("[warning] jointTols->size() != axes\n");
    } else {
        jointTols = 0;
        printf("FakeControlboard not using individual jointTols, defaulting to genJointTol.\n");
    }
    yarp::os::Bottle* maxLimits;
    if (config.check("maxLimits")) {
        maxLimits = config.find("maxLimits").asList();
        printf("FakeControlboard using individual maxLimits: %s\n",maxLimits->toString().c_str());
        if(maxLimits->size() != axes) printf("[warning] maxLimits->size() != axes\n");
    } else {
        maxLimits = 0;
        printf("FakeControlboard not using individual maxLimits, defaulting to genMaxLimit.\n");
    }
    yarp::os::Bottle* minLimits;
    if (config.check("minLimits")) {
        minLimits = config.find("minLimits").asList();
        printf("FakeControlboard using individual minLimits: %s\n",minLimits->toString().c_str());
        if(minLimits->size() != axes) printf("[warning] minLimits->size() != axes\n");
    } else {
        minLimits = 0;
        printf("FakeControlboard not using individual minLimits, defaulting to genMinLimit.\n");
    }
    yarp::os::Bottle* refSpeeds;
    if (config.check("refSpeeds")) {
        refSpeeds = config.find("refSpeeds").asList();
        printf("FakeControlboard using individual refSpeeds: %s\n",refSpeeds->toString().c_str());
        if(refSpeeds->size() != axes) printf("[warning] refSpeeds->size() != axes\n");
    } else {
        refSpeeds = 0;
        printf("FakeControlboard not using individual refSpeeds, defaulting to genRefSpeed.\n");
    }
    yarp::os::Bottle* encRawExposeds;
    if (config.check("encRawExposeds")) {
        encRawExposeds = config.find("encRawExposeds").asList();
        printf("FakeControlboard using individual encRawExposeds: %s\n",encRawExposeds->toString().c_str());
        if(encRawExposeds->size() != axes) printf("[warning] encRawExposeds->size() != axes\n");
    } else {
        encRawExposeds = 0;
        printf("FakeControlboard not using individual encRawExposeds, defaulting to genEncRawExposed.\n");
    }
    yarp::os::Bottle* velRawExposeds;
    if (config.check("velRawExposeds")) {
        velRawExposeds = config.find("velRawExposeds").asList();
        printf("FakeControlboard using individual velRawExposeds: %s\n",velRawExposeds->toString().c_str());
        if(velRawExposeds->size() != axes) printf("[warning] velRawExposeds->size() != axes\n");
    } else {
        velRawExposeds = 0;
        printf("FakeControlboard not using individual velRawExposeds, defaulting to genVelRawExposed.\n");
    }

    encRawExposed.resize(axes);
    jointStatus.resize(axes);
    initPos.resize(axes);
    jointTol.resize(axes);
    maxLimit.resize(axes);
    minLimit.resize(axes);
    refSpeed.resize(axes);
    velRawExposed.resize(axes);
    for (unsigned int i=0; i<axes; i++) {
        jointStatus[i]=0;
        if(!refSpeeds) refSpeed[i]=genRefSpeed;
        else refSpeed[i]=refSpeeds->get(i).asDouble();
        if(!minLimits) minLimit[i]=genMinLimit;
        else minLimit[i]=minLimits->get(i).asDouble();
        if(!maxLimits) maxLimit[i]=genMaxLimit;
        else maxLimit[i]=maxLimits->get(i).asDouble(); 
        if(!initPoss) initPos[i]=genInitPos;
        else initPos[i]=initPoss->get(i).asDouble(); 
        if(!jointTols) jointTol[i]=genJointTol;
        else jointTol[i]=jointTols->get(i).asDouble(); 
        if(!encRawExposeds) encRawExposed[i]=genEncRawExposed;
        else encRawExposed[i]=encRawExposeds->get(i).asDouble(); 
        if(!velRawExposeds) velRawExposed[i]=genVelRawExposed;
        else velRawExposed[i]=velRawExposeds->get(i).asDouble(); 
    }
    encRaw.resize(axes, 0.0);
    refAcc.resize(axes, 1.0);
    targetExposed.resize(axes, 0.0);
    velRaw.resize(axes, 0.0);

    for (unsigned int i=0; i<axes; i++) {
        setEncoder(i,initPos[i]);
    }

    // Start the RateThread
    this->setRate(jmcMs);
    this->start();
    
    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeControlboard::close() {
    printf("[FakeControlboard] close()\n");
    return true;
}

// -----------------------------------------------------------------------------


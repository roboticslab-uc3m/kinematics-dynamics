// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoGravityCompensator.hpp"

/************************************************************************/
bool teo::TeoGravityCompensator::configure(yarp::os::ResourceFinder &rf) {

    std::string solver = rf.check("solver",yarp::os::Value(DEFAULT_SOLVER),"solver device type").asString();
    std::string kinematics = rf.check("kinematics",yarp::os::Value(DEFAULT_KINEMATICS),"limb kinematic description").asString();
    std::string remote = rf.check("remote",yarp::os::Value(DEFAULT_REMOTE),"remote robot").asString();
    if( rf.check("help") ) {
        CD_INFO("Using solver: %s\n",solver.c_str());
    }

    std::string kinematicsFull("../kinematics/");
    kinematicsFull += kinematics;
    std::string ini = rf.findFileByName( kinematicsFull );

    yarp::os::Property solverOptions;
    if (! solverOptions.fromConfigFile(ini) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"%s\".\n",kinematics.c_str());
        return false;
    }
    solverOptions.put("device",solver);
    solverDevice.open(solverOptions);

    if (!solverDevice.isValid()) {
        CD_ERROR("solver device instantiation not worked.\n");
        // solverDevice.close();  // un-needed?
        return false;
    }

    if ( ! solverDevice.view( gravityRateThread.solver ) ) {
        CD_ERROR("Could not obtain solver interface.\n");
        return false;
    }

    //-- robot arm device (remote) --
    std::string local("/teoGravityCompensator");
    local += remote;
    yarp::os::Property robotOptions;
    robotOptions.put("device","remote_controlboard");
    robotOptions.put("local",local);
    robotOptions.put("remote",remote);

    robotDevice.open(robotOptions);

    if (!robotDevice.isValid()) {
        CD_ERROR("robotDevice instantiation not worked.\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    if ( ! robotDevice.view( gravityRateThread.iEncoders ) ) {
        CD_ERROR("Could not obtain right arm encoders interface.\n");
        return false;
    }

    if ( ! robotDevice.view( gravityRateThread.iTorqueControl ) ) {
        CD_ERROR("Could not obtain right arm torque interface.\n");
        return false;
    }

    //-- Start the thread.
    return gravityRateThread.start();
}

/************************************************************************/
bool teo::TeoGravityCompensator::updateModule() {
    //CD_INFO("Alive\n");
    return true;
}

/************************************************************************/
bool teo::TeoGravityCompensator::interruptModule() {

    gravityRateThread.stop();

    robotDevice.close();
    solverDevice.close();

    return true;
}

/************************************************************************/


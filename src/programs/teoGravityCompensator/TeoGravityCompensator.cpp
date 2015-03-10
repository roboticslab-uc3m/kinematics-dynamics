// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoGravityCompensator.hpp"

/************************************************************************/
bool teo::TeoGravityCompensator::configure(yarp::os::ResourceFinder &rf) {

    std::string solver = rf.check("solver",yarp::os::Value(DEFAULT_SOLVER),"solver device type").asString();
    if( rf.check("help") ) {
        CD_INFO("Using solver: %s\n",solver.c_str());
    }

    //-- canId 22 (left arm shoulder) solver --
    std::string ini = rf.findFileByName("../kinematics/leftArm22Kinematics.ini");

    yarp::os::Property solverOptions;
    if (! solverOptions.fromConfigFile(ini) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"rightArmKinematics.ini\".\n");
        return false;
    }
    solverOptions.put("device",solver);
    solverDevice.open(solverOptions);

    if (!solverDevice.isValid()) {
        CD_ERROR("Right arm solver device instantiation not worked.\n");
        // rightArmSolverDevice.close();  // un-needed?
        return false;
    }

    if ( ! solverDevice.view( gravityRateThread.solver ) ) {
        CD_ERROR("Could not obtain solver interface.\n");
        return false;
    }

    //-- robot arm device (remote) --
    yarp::os::Property robotOptions;
    robotOptions.put("device","remote_controlboard");
    robotOptions.put("local","/teoGravityCompensator");
    robotOptions.put("remote","/testBodyBot");

    robotDevice.open(robotOptions);

    if (!robotDevice.isValid()) {
        CD_ERROR("robotDevice instantiation not worked.\n");
        // robotDeviceRA.close();  // un-needed?
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


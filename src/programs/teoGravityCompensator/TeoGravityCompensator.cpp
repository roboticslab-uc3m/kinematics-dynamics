// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoGravityCompensator.hpp"

/************************************************************************/
bool teo::TeoGravityCompensator::configure(yarp::os::ResourceFinder &rf) {

    std::string solver = rf.check("solver",yarp::os::Value(DEFAULT_SOLVER),"solver device type").asString();
    if( rf.check("help") ) {
        CD_INFO("Using solver: %s\n",solver.c_str());
    }

    //-- full right arm solver --
    //std::string iniRA = rf.findFileByName("../kinematics/rightArmKinematics.ini");

    //-- id22 left arm solver --
    std::string iniRA = rf.findFileByName("../kinematics/leftArm22Kinematics.ini");

    yarp::os::Property solverOptionsRA;
    if (! solverOptionsRA.fromConfigFile(iniRA) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"rightArmKinematics.ini\".\n");
        return false;
    }
    solverOptionsRA.put("device",solver);
    solverDeviceRA.open(solverOptionsRA);

    if (!solverDeviceRA.isValid()) {
        CD_ERROR("Right arm solver device instantiation not worked.\n");
        // rightArmSolverDevice.close();  // un-needed?
        return false;
    }

    if ( ! solverDeviceRA.view( gravityRateThread.solverRA ) ) {
        CD_ERROR("Could not obtain solver interface.\n");
        return false;
    }

    //-- robot arm device (remote) --
    yarp::os::Property robotOptionsRA;
    robotOptionsRA.put("device","remote_controlboard");
    robotOptionsRA.put("local","/teoGravityCompensator");
    robotOptionsRA.put("remote","/testBodyBot");

    robotDeviceRA.open(robotOptionsRA);

    if (!robotDeviceRA.isValid()) {
        CD_ERROR("robotDeviceRA instantiation not worked.\n");
        // robotDeviceRA.close();  // un-needed?
        return false;
    }

    if ( ! robotDeviceRA.view( gravityRateThread.iEncodersRA ) ) {
        CD_ERROR("Could not obtain right arm encoders interface.\n");
        return false;
    }

    if ( ! robotDeviceRA.view( gravityRateThread.iTorqueControlRA ) ) {
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

    robotDeviceRA.close();
    solverDeviceRA.close();

    return true;
}

/************************************************************************/


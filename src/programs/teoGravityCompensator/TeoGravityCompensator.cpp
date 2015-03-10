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

    //-- full right arm device (remote) --
    yarp::os::Property robotOptionsRA;
    robotOptionsRA.put("device","remote_controlboard");
    robotOptionsRA.put("local","/teoGravityCompensator/rightArm");
    robotOptionsRA.put("remote","/teoSim/rightArm");

    //-- id22 left arm device (local) --
    /*yarp::os::Property robotOptionsRA;
    robotOptionsRA.put("device","bodybot");
    robotOptionsRA.put("mode","torque");
    robotOptionsRA.put("canDevice","/dev/can1");
    robotOptionsRA.put("types","motoripos");
    robotOptionsRA.put("ids",22);
    robotOptionsRA.put("maxs",360);
    robotOptionsRA.put("mins",-360);
    robotOptionsRA.put("ks",0.0706);
    robotOptionsRA.put("refAccelerations",0.575437);
    robotOptionsRA.put("refSpeeds",737.2798);
    robotOptionsRA.put("trs",160);*/

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


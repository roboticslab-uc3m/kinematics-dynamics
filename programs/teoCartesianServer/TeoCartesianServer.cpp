// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoCartesianServer.hpp"

namespace teo
{

/************************************************************************/
TeoCartesianServer::TeoCartesianServer() { }

/************************************************************************/
bool TeoCartesianServer::configure(ResourceFinder &rf) {

    std::string solver = rf.check("solver",yarp::os::Value(DEFAULT_SOLVER),"solver device type").asString();
    std::string kinematics = rf.check("kinematics",yarp::os::Value(DEFAULT_KINEMATICS),"limb kinematic description").asString();
    std::string remote = rf.check("remote",yarp::os::Value(DEFAULT_REMOTE),"remote robot").asString();
    std::string angleRepr = rf.check("angleRepr",yarp::os::Value(DEFAULT_ANG_REPR),"angle representation").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("TeoCartesianServer options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--solver (solver device type, default: \"%s\")\n",DEFAULT_SOLVER);
        printf("\t--kinematics (limb kinematic description, default: \"%s\")\n",DEFAULT_KINEMATICS);
        printf("\t--remote (port to whom we connect, default: \"%s\")\n",DEFAULT_REMOTE);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    printf("TeoCartesianServer using solver: %s,  kinematics: %s, remote: %s.\n",solver.c_str(),kinematics.c_str(),remote.c_str());

    //------------------------------CARTESIAN--------------------------------//
    std::string kinematicsFull("../kinematics/");
    kinematicsFull += kinematics;
    std::string ini = rf.findFileByName( kinematicsFull );

    yarp::os::Property solverOptions;
    if (! solverOptions.fromConfigFile(ini) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"%s\".\n",ini.c_str());
        return false;
    }
    solverOptions.put("device",solver);
    solverOptions.put("angleRepr",angleRepr);
    solverDevice.open(solverOptions);

    if (!solverDevice.isValid()) {
        CD_ERROR("solver device instantiation not worked.\n");
        // solverDevice.close();  // un-needed?
        return false;
    }

    if ( ! solverDevice.view( cartesianRateThread.solver ) ) {
        CD_ERROR("Could not obtain solver interface.\n");
        return false;
    }

    //--------------------------------JOINT----------------------------------//
    Property robotOptions;
    robotOptions.put("device","remote_controlboard");
    robotOptions.put("remote",remote);
    std::string local("/teoCartesianServer");
    local += remote;
    robotOptions.put("local",local);
    robotDevice.open(robotOptions);
    if (!robotDevice.isValid()) {
        CD_ERROR("Could not open robot device: remote_controlboard\n\n");
        CD_ERROR("device not valid, has a remotely accessible device been open?\n\n");
        return false;
    }
    if ( ! robotDevice.view( cartesianRateThread.iEncoders ) ) {
        CD_ERROR("Could not obtain iEncoders.\n");
        return false;
    }
    CD_SUCCESS("Obtained iEncoders.\n");

    if ( ! robotDevice.view( cartesianRateThread.iPositionControl ) ) {
        CD_ERROR("Could not obtain iPositionControl.\n");
        return false;
    }
    CD_SUCCESS("Obtained iPositionControl.\n");

    if ( ! robotDevice.view( cartesianRateThread.iVelocityControl ) ) {
        CD_ERROR("Could not obtain iVelocityControl.\n");
        return false;
    }
    CD_SUCCESS("Obtained iVelocityControl.\n");

    //---------------------CONFIGURE PORTs------------------------
    xResponder.setCartesianRateThread(&cartesianRateThread);
    cartesianRateThread.setRf(&rf);
    cartesianRateThread.start();
    std::string xRpcServerStr(local);
    xRpcServerStr += "/rpc:i";
    xRpcServer.open(xRpcServerStr);
    xRpcServer.setReader(xResponder);
    std::string xPortStr(local);
    xPortStr += "/command:i";
    xPort.open(xPortStr);
    xPort.useCallback();
    return true;
}

/************************************************************************/
bool TeoCartesianServer::updateModule() {
    // printf("Alive\n");
    return true;
}

/************************************************************************/
bool TeoCartesianServer::interruptModule() {
    cartesianRateThread.stop();
    xRpcServer.interrupt();
    xPort.disableCallback();
    solverDevice.close();
    xRpcServer.close();
    xPort.close();
    return true;
}

/************************************************************************/

}  // namespace teo

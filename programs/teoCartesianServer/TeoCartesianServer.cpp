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
    std::string prefix = DEFAULT_PREFIX;
    std::string movjLocal = DEFAULT_MOVJ_LOCAL;
    std::string movjRemote = DEFAULT_MOVJ_REMOTE;
    csStatus = 0;

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("TeoCartesianServer options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--solver (solver device type, default: \"%s\")\n",DEFAULT_SOLVER);
        printf("\t--kinematics (limb kinematic description, default: \"%s\")\n",DEFAULT_KINEMATICS);
        printf("\t--prefix (port name prefix, default: \"%s\")\n",prefix.c_str());
        printf("\t--movjLocal (port we open to connect for movj, default: \"%s\")\n",movjLocal.c_str());
        printf("\t--movjRemote (port to whom we connect for movj, default: \"%s\")\n",movjRemote.c_str());
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("prefix")) prefix = rf.find("prefix").asString();
    if (rf.check("movjRemote")) movjRemote = rf.find("movjRemote").asString();
    if (rf.check("movjLocal")) movjLocal = rf.find("movjLocal").asString();
    printf("TeoCartesianServer using solver: %s,  kinematics: %s, prefix: %s.\n",solver.c_str(),kinematics.c_str(),prefix.c_str());
    printf("TeoCartesianServer using movjLocal: %s, movjRemote: %s.\n",movjLocal.c_str(),movjRemote.c_str());

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
    solverDevice.open(solverOptions);

    if (!solverDevice.isValid()) {
        CD_ERROR("solver device instantiation not worked.\n");
        // solverDevice.close();  // un-needed?
        return false;
    }

    /*if ( ! solverDevice.view( cartesianRateThread.solver ) ) {
        CD_ERROR("Could not obtain solver interface.\n");
        return false;
    }*/

    //--------------------------------JOINT----------------------------------//
    Property robotOptions;
    robotOptions.fromString(rf.toString());  // Get rf stuff to the module
    robotOptions.put("device","remote_controlboard");
    robotOptions.put("remote",movjRemote);
    robotOptions.put("local",movjLocal);
    robotDevice.open(robotOptions);
    if (!robotDevice.isValid()) {
        CD_ERROR("Could not open robot device: remote_controlboard\n\n");
        CD_ERROR("movjRemote not valid, has a remotely accessible device been open?\n\n");
        return false;
    }
    bool ok2 = robotDevice.view(ipos);
    if (!ok2) {
        fprintf(stderr, "[CartesianServer] warning: Problems acquiring robot interfaces.\n");
        return false;
    } else printf("[CartesianServer] success: Acquired robot interfaces.\n");

    //---------------------CONFIGURE PORTs------------------------
    xResponder.setPositionInterface(ipos);
    xResponder.setCsStatus(&csStatus);
    ConstString xRpcServerStr(prefix);
    xRpcServerStr += "/cartesianServer/rpc:i";
    xRpcServer.open(xRpcServerStr);
    xRpcServer.setReader(xResponder);
    xPort.setPositionInterface(ipos);
    xPort.setCsStatus(&csStatus);
    ConstString xPortStr(prefix);
    xPortStr += "/cartesianServer/command:i";
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
    xRpcServer.interrupt();
    xPort.disableCallback();
    solverDevice.close();
    xRpcServer.close();
    xPort.close();
    return true;
}

/************************************************************************/

}  // namespace teo

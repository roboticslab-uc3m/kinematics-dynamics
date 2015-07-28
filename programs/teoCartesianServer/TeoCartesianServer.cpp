// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoCartesianServer.hpp"

namespace teo
{

/************************************************************************/
TeoCartesianServer::TeoCartesianServer() { }

/************************************************************************/
bool TeoCartesianServer::configure(ResourceFinder &rf) {

    std::string controller = DEFAULT_CONTROLLER;
    std::string prefix = DEFAULT_PREFIX;
    std::string movjLocal = DEFAULT_MOVJ_LOCAL;
    std::string movjRemote = DEFAULT_MOVJ_REMOTE;
    csStatus = 0;

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("CartesianServer options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--controller (cartesian controller device, default: \"%s\")\n",controller.c_str());
        printf("\t--prefix (port name prefix, default: \"%s\")\n",prefix.c_str());
        printf("\t--movjLocal (port we open to connect for movj, default: \"%s\")\n",movjLocal.c_str());
        printf("\t--movjRemote (port to whom we connect for movj, default: \"%s\")\n",movjRemote.c_str());
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("controller")) controller = rf.find("controller").asString();
    if (rf.check("prefix")) prefix = rf.find("prefix").asString();
    if (rf.check("movjRemote")) movjRemote = rf.find("movjRemote").asString();
    if (rf.check("movjLocal")) movjLocal = rf.find("movjLocal").asString();
    printf("CartesianServer using controller: %s,  prefix: %s.\n",controller.c_str(),prefix.c_str());
    printf("CartesianServer using movjLocal: %s, movjRemote: %s.\n",movjLocal.c_str(),movjRemote.c_str());

    //------------------------------CARTESIAN--------------------------------//
    Property options;
    options.fromString(rf.toString());  // Get rf stuff to the cartesian device
    options.put("device",controller);
    cartesianDevice.open(options);
    if (!cartesianDevice.isValid()) {
        CD_ERROR("Could not open controller: %s\n",controller.c_str());
        printf("Be sure CMake \"ENABLE_RlPlugins_%s\" variable is set \"ON\"\n",controller.c_str());
        printf("\"SKIP_%s is set\" --> should be --> \"ENABLE_%s is set\"\n\n",controller.c_str(),controller.c_str());
        return false;
    }
    bool ok = cartesianDevice.view(icart);
    if (!ok) {
        fprintf(stderr, "[CartesianServer] warning: Problems acquiring cartesian interface.\n");
        return false;
    } else printf("[CartesianServer] success: Acquired cartesian interface.\n");

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
    xResponder.setCartesianInterface(icart);
    xResponder.setCsStatus(&csStatus);
    ConstString xRpcServerStr(prefix);
    xRpcServerStr += "/cartesianServer/rpc:i";
    xRpcServer.open(xRpcServerStr);
    xRpcServer.setReader(xResponder);
    xPort.setPositionInterface(ipos);
    xPort.setCartesianInterface(icart);
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
    cartesianDevice.close();
    xRpcServer.close();
    xPort.close();
    return true;
}

/************************************************************************/

}  // namespace teo

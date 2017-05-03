// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Cv1ToRoot.hpp"

namespace teo
{

/************************************************************************/
bool Cv1ToRoot::updateModule() {
    printf("Cv1ToRoot alive...\n");
    return true;
}

/************************************************************************/
double Cv1ToRoot::getPeriod() {
    return watchdog;  // [s]
}

/************************************************************************/

bool Cv1ToRoot::configure(yarp::os::ResourceFinder &rf) {
    watchdog = DEFAULT_WATCHDOG;  // double

    fprintf(stdout,"--------------------------------------------------------------\n");
    if(rf.check("help")) {
       printf("Cv1ToRoot Options:\n");
       printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);
    }
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asDouble();
    fprintf(stdout,"Cv1ToRoot using watchdog [s]: %f.\n",watchdog);

    fprintf(stdout,"--------------------------------------------------------------\n");
    if(rf.check("help")) {
       return false;
    }

    outPort.open("/out");
    premultPorts.setOutPort(&outPort);
    premultPorts.open("/in");
    premultPorts.useCallback();
    return true;
}

/************************************************************************/

bool Cv1ToRoot::interruptModule() {
    premultPorts.disableCallback();
    premultPorts.close();
    outPort.close();
    return true;
}

/************************************************************************/

}  // namespace teo

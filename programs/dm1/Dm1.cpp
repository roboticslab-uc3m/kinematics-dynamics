// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Dm1.hpp"

/************************************************************************/

bool Dm1::configure(ResourceFinder &rf) {

    //ConstString fileName(DEFAULT_FILE_NAME);
    
    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Dm1 options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        //printf("\t--file (default: \"%s\")\n",fileName.c_str());
    }
    //if (rf.check("file")) fileName = rf.find("file").asString();
    //printf("Dm1 using file: %s\n",fileName.c_str());

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //-----------------OPEN LOCAL PORTS------------//
    outCmdPort.open("/dm1/command:o");
    outTtsPort.open("/dm1/tts:o");
    inSrPort.open("/dm1/sr:i");
    stateMachine.setOutCmdPort(&outCmdPort);
    stateMachine.setOutTtsPort(&outTtsPort);
    stateMachine.setInSrPort(&inSrPort);
    while(1){
        if(outTtsPort.getOutputCount() > 0) break;
        printf("Waiting for \"/dm1/tts:o\" to be connected to something...\n");
        Time::delay(0.5);
    }    
    stateMachine.start();
    return true;
}

/************************************************************************/
double Dm1::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool Dm1::updateModule() {
    printf("StateMachine in state [%d]. Dm1 alive...\n", stateMachine.getMachineState());
    return true;
}

/************************************************************************/

bool Dm1::interruptModule() {
    printf("Dm1 closing...\n");
    inSrPort.interrupt();
    outTtsPort.interrupt();
    stateMachine.stop();
    inSrPort.close();
    outTtsPort.close();
    return true;
}

/************************************************************************/


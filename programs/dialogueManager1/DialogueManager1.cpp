// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DialogueManager1.hpp"

/************************************************************************/

bool DialogueManager1::configure(ResourceFinder &rf) {

    //ConstString fileName(DEFAULT_FILE_NAME);
    
    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("DialogueManager1 options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        //printf("\t--file (default: \"%s\")\n",fileName.c_str());
    }
    //if (rf.check("file")) fileName = rf.find("file").asString();
    //printf("DialogueManager1 using file: %s\n",fileName.c_str());

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //-----------------OPEN LOCAL PORTS------------//
    outCmdPort.open("/dialogueManager1/command:o");
    outTtsPort.open("/dialogueManager1/tts:o");
    inSrPort.open("/dialogueManager1/sr:i");
    stateMachine.setOutCmdPort(&outCmdPort);
    stateMachine.setOutTtsPort(&outTtsPort);
    stateMachine.setInSrPort(&inSrPort);
    while(1){
        if(outTtsPort.getOutputCount() > 0) break;
        printf("Waiting for \"/dialogueManager1/tts:o\" to be connected to something...\n");
        Time::delay(0.5);
    }    
    stateMachine.start();
    return true;
}

/************************************************************************/
double DialogueManager1::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool DialogueManager1::updateModule() {
    printf("StateMachine in state [%d]. DialogueManager1 alive...\n", stateMachine.getMachineState());
    return true;
}

/************************************************************************/

bool DialogueManager1::interruptModule() {
    printf("DialogueManager1 closing...\n");
    inSrPort.interrupt();
    outTtsPort.interrupt();
    stateMachine.stop();
    inSrPort.close();
    outTtsPort.close();
    return true;
}

/************************************************************************/


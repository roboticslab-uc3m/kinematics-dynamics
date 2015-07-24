// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DM1_HPP__
#define __DM1_HPP__

#include <yarp/os/all.h>
#include <stdlib.h>

#include "StateMachine.hpp"

//j//#define DEFAULT_FILE_NAME "segRec_ecf_params.xml"

#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

using namespace yarp::os;

class Dm1 : public RFModule {
  private:
    StateMachine stateMachine;
    BufferedPort<Bottle> inSrPort;
    Port outTtsPort;
    Port outCmdPort;

    bool interruptModule();
    double getPeriod();
    bool updateModule();

  public:
    bool configure(ResourceFinder &rf);
};

#endif  // __DM1_HPP__

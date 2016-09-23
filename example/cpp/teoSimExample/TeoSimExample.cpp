// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoSimExample.hpp"

namespace teo
{

bool TeoSimExample::run(){

    printf("WARNING: requires a running instance of teoSim\n");
    if (!yarp::os::Network::checkNetwork()) {
        printf("Please start a yarp name server first\n");
        return(-1);
    }

    //Configure Drivers
    options.put("device","remote_controlboard"); //we add a name-value pair that indicates the YARP device
    options.put("remote","/teoSim/rightArm"); //we add info on to whom we will connect
    options.put("local","/local"); //we add info on how we will call ourselves on the YARP network
    dd.open(options); //Configure the YARP multi-use driver with the given options

    if(!dd.isValid()) {
      printf("/teoSim/rightArm device not available.\n");
	  dd.close();
      yarp::os::Network::fini(); //disconnect from the YARP network
      return 1;
    }

    bool ok = dd.view(pos);
    if (!ok) {
        printf("[warning] Problems acquiring robot interface\n");
        return false;
    } else printf("[success] testAsibot acquired robot interface\n");

    pos->setPositionMode(); //use the object to set the device to position mode (as opposed to velocity mode)

    printf("test positionMove(1,-35)\n");
    pos->positionMove(1, -35);

    printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5);

    ok = dd.view(enc);

    ok = dd.view(vel);
    vel->setVelocityMode(); //use the object to set the device to velocity mode (as opposed to position mode)
    printf("test velocityMove(0,10)\n");
    vel->velocityMove(0,10);

    printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5);

    return 0;
}

} //namespace TEO

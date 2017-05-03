#include "TransCoordsUsingJoints.hpp"

namespace teo
{

/************************************************************************/
bool TransCoordsUsingJoints::updateModule()
{
    printf("TransCoordsUsingJoints alive...\n");
    return true;
}

/************************************************************************/
double TransCoordsUsingJoints::getPeriod()
{
    return watchdog;  // [s]
}

/************************************************************************/

bool TransCoordsUsingJoints::configure(yarp::os::ResourceFinder &rf)
{
    watchdog = DEFAULT_WATCHDOG;  // double

    fprintf(stdout,"--------------------------------------------------------------\n");
    if( rf.check("help") )
    {
       printf("TransCoordsUsingJoints Options:\n");
       printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);
    }
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asDouble();
    fprintf(stdout,"TransCoordsUsingJoints using watchdog [s]: %f.\n",watchdog);

    fprintf(stdout,"--------------------------------------------------------------\n");
    if(rf.check("help"))
    {
       return false;
    }

    yarp::os::Property robotOptions;
    robotDevice.open(robotOptions);
    if( ! robotDevice.isValid() ) {
        CD_ERROR("robot device not valid.\n");
        return false;
    }
    yarp::dev::IEncoders* iEncoders;
    if( ! robotDevice.view(iEncoders) ) {
        CD_ERROR("Could not view iEncoders.\n");
        return false;
    }

    outPort.open("/out");
    premultPorts.setOutPort(&outPort);
    premultPorts.setIEncoders(iEncoders);
    premultPorts.open("/in");
    premultPorts.useCallback();

    return true;
}

/************************************************************************/

bool TransCoordsUsingJoints::interruptModule()
{
    premultPorts.disableCallback();
    premultPorts.close();
    outPort.close();
    robotDevice.close();
    return true;
}

/************************************************************************/

}  // namespace teo

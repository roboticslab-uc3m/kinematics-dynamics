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

    outPort.open("/out");
    premultPorts.setOutPort(&outPort);
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
    return true;
}

/************************************************************************/

}  // namespace teo

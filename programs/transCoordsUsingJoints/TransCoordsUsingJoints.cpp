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
    return 2.0;  // [s]
}

/************************************************************************/

bool TransCoordsUsingJoints::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("config: %s.\n", rf.toString().c_str());

    if(rf.check("help"))
    {
        printf("OneCanBusOneWrapper options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
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

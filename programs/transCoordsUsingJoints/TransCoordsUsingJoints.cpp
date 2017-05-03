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
    CD_DEBUG("TransCoordsUsingJoints config: %s.\n", rf.toString().c_str());

    std::string solverStr = rf.check("solver",yarp::os::Value(DEFAULT_SOLVER),"cartesian solver").asString();
    std::string robotStr = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"robot device").asString();

    if(rf.check("help"))
    {
        printf("TransCoordsUsingJoints options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        return false;
    }

    yarp::os::Property solverOptions;
    solverOptions.fromString( rf.toString() );
    solverOptions.put("device",solverStr);

    solverDevice.open(solverOptions);
    if( ! solverDevice.isValid() ) {
        CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        return false;
    }

    yarp::os::Property robotOptions;
    robotOptions.fromString( rf.toString() );
    robotOptions.put("device",robotStr);
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

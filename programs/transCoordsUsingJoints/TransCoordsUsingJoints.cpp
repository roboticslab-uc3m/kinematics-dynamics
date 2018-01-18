#include "TransCoordsUsingJoints.hpp"

#include <string>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <yarp/dev/IEncoders.h>

#include <ColorDebug.hpp>

#include "ICartesianSolver.h"

namespace roboticslab
{

/************************************************************************/
bool TransCoordsUsingJoints::updateModule()
{
    CD_INFO("TransCoordsUsingJoints alive...\n");
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
    CD_INFO("TransCoordsUsingJoints config: %s.\n", rf.toString().c_str());

    std::string solverStr = rf.check("solver",yarp::os::Value(DEFAULT_SOLVER),"cartesian solver").asString();
    std::string robotStr = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"robot device").asString();

    if(rf.check("help"))
    {
        CD_INFO_NO_HEADER("TransCoordsUsingJoints options:\n");
        CD_INFO_NO_HEADER("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
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
    roboticslab::ICartesianSolver* iCartesianSolver;
    if( ! solverDevice.view(iCartesianSolver) ) {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
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
    int numRobotJoints;
    if( ! iEncoders->getAxes(&numRobotJoints) )
    {
        CD_ERROR("Could not get axes.\n");
        return false;
    }
    CD_SUCCESS("numRobotJoints: %d.\n",numRobotJoints);

    outPort.open("/coords:o");
    premultPorts.setOutPort(&outPort);
    premultPorts.setIEncoders(iEncoders);
    premultPorts.setICartesianSolver(iCartesianSolver);
    premultPorts.setNumRobotJoints(numRobotJoints);
    premultPorts.open("/coords:i");
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

}  // namespace roboticslab

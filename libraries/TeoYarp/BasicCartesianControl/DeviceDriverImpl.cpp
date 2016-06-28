// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::BasicCartesianControl::open(yarp::os::Searchable& config) {

    CD_DEBUG("BasicCartesianControl config: %s.\n", config.toString().c_str());

    std::string solverStr = config.check("solver",yarp::os::Value(DEFAULT_SOLVER),"cartesian solver").asString();
    std::string robotStr = config.check("robot",yarp::os::Value(DEFAULT_ROBOT),"robot device").asString();

    yarp::os::Property solverOptions;
    solverOptions.fromString( config.toString() );
    solverOptions.put("device",solverStr);

    solverDevice.open(solverOptions);
    if( ! solverDevice.isValid() ) {
        CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        return false;
    }
    if( ! solverDevice.view(iCartesianSolver) ) {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;
    }

    yarp::os::Property robotOptions;
    robotOptions.fromString( config.toString() );
    robotOptions.put("device",robotStr);
    robotDevice.open(robotOptions);
    if( ! robotDevice.isValid() ) {
        CD_ERROR("robot device not valid: %s.\n",robotStr.c_str());
        return false;
    }
    if( ! robotDevice.view(iEncoders) ) {
        CD_ERROR("Could not view iEncoders in: %s.\n",robotStr.c_str());
        return false;
    }
    iEncoders->getAxes(&numRobotJoints);
    CD_INFO("numRobotJoints: %d.\n",numRobotJoints);

    iCartesianSolver->getNumLinks( &numSolverLinks );
    CD_INFO("numSolverLinks: %d.\n",numSolverLinks);

    if( numRobotJoints != numSolverLinks )
    {
        CD_WARNING("numRobotJoints(%d) != numSolverLinks(%d) !!!\n",numRobotJoints,numSolverLinks);
    }

    return this->start();
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::close() {

    return true;
}

// -----------------------------------------------------------------------------

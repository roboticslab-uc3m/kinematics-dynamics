// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::CartesianControlServer::open(yarp::os::Searchable& config) {

    /*CD_DEBUG("CartesianControlServer config: %s.\n", config.toString().c_str());

    std::string solverStr = config.check("solver",yarp::os::Value(DEFAULT_SOLVER),"cartesian solver").asString();
    std::string robotStr = config.check("robot",yarp::os::Value(DEFAULT_ROBOT),"robot device").asString();

    yarp::os::Property solverOptions;
    solverOptions.fromString( config.toString() );
    solverOptions.put("device",solverStr);

    solverDevice.open(solverOptions);
    if( ! solverDevice.isValid() ) {
        CD_ERROR("solverDevice not valid: %s.\n",solverStr.c_str());
        return false;
    }
    if( ! solverDevice.view(iCartesianSolver) ) {
        CD_ERROR("Could not view ICartesianSolver in: %s.\n",solverStr.c_str());
        return false;
    }

    yarp::os::Property robotOptions;
    robotOptions.fromString( config.toString() );
    robotOptions.put("device",robotStr);
    robotDevice.open(robotOptions);
    if( ! robotDevice.isValid() ) {
        CD_ERROR("robotDevice not valid: %s.\n",robotStr.c_str());
        return false;
    }
    if( ! robotDevice.view(iEncoders) ) {
        CD_ERROR("Could not view iEncoders in: %s.\n",robotStr.c_str());
        return false;
    }
    iEncoders->getAxes(&numRobotJoints);*/

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CartesianControlServer::close() {

    return true;
}

// -----------------------------------------------------------------------------

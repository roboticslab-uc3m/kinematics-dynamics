// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::BasicCartesianControl::open(yarp::os::Searchable& config) {

    std::string solverStr = config.check("solver",yarp::os::Value(DEFAULT_SOLVER),"cartesian solver").asString();

    yarp::os::Property solverOptions;
    solverOptions.fromString( config.toString() );
    solverOptions.put("device",solverStr);

    CD_DEBUG("solverOptions: %s.\n", solverOptions.toString().c_str());
    solverDevice.open(solverOptions);
    if( ! solverDevice.isValid() ) {
        CD_ERROR("solverDevice not valid: %s.\n",solverStr.c_str());
        return false;
    }
    if( ! solverDevice.view(iCartesianSolver) ) {
        CD_ERROR("Could not view ICartesianSolver in: %s.\n",solverStr.c_str());
        return false;
    }


    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::close() {

    return true;
}

// -----------------------------------------------------------------------------

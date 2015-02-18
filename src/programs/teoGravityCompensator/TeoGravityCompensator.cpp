// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoGravityCompensator.hpp"

/************************************************************************/
bool teo::TeoGravityCompensator::configure(yarp::os::ResourceFinder &rf) {

    std::string solver = rf.check("solver",yarp::os::Value(DEFAULT_SOLVER),"solver device type").asString();
    if( rf.check("help") ) {
        CD_INFO("Using solver: %s\n",solver.c_str());
    }

    //-- right arm --
    std::string rightArmIni = rf.findFileByName("../kinematics/rightArmKinematics.ini");

    yarp::os::Property rightArmSolverOptions;
    if (! rightArmSolverOptions.fromConfigFile(rightArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"rightArmKinematics.ini\".\n");
        return false;
    }
    rightArmSolverOptions.put("device",solver);
    rightArmSolverDevice.open(rightArmSolverOptions);

    if (!rightArmSolverDevice.isValid()) {
        CD_ERROR("rightArmSolverDevice instantiation not worked.\n");
        // rightArmSolverDevice.close();  // un-needed?
        return false;
    }

    return true;
}

/************************************************************************/
bool teo::TeoGravityCompensator::updateModule() {
    CD_INFO("Alive\n");
    return true;
}

/************************************************************************/
bool teo::TeoGravityCompensator::interruptModule() {
    return true;
}

/************************************************************************/


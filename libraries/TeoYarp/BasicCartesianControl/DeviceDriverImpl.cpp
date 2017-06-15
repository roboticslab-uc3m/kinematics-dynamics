// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::BasicCartesianControl::open(yarp::os::Searchable& config) {

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
    if( ! robotDevice.view(iPositionControl) ) {
        CD_ERROR("Could not view iPositionControl in: %s.\n",robotStr.c_str());
        return false;
    }
    if( ! robotDevice.view(iVelocityControl) ) {
        CD_ERROR("Could not view iVelocityControl in: %s.\n",robotStr.c_str());
        return false;
    }
    if( ! robotDevice.view(iControlLimits) ) {
        CD_ERROR("Could not view iControlLimits in: %s.\n",robotStr.c_str());
        return false;
    }
    if( ! robotDevice.view(iTorqueControl) ) {
        CD_ERROR("Could not view iTorqueControl in: %s.\n",robotStr.c_str());
        return false;
    }
    if( ! robotDevice.view(iControlMode) ) {
        CD_ERROR("Could not view iControlMode in: %s.\n",robotStr.c_str());
        return false;
    }

    iEncoders->getAxes(&numRobotJoints);
    CD_INFO("numRobotJoints: %d.\n",numRobotJoints);

    iCartesianSolver->getNumJoints( &numSolverJoints );
    CD_INFO("numSolverJoints: %d.\n",numSolverJoints);

    if( numRobotJoints != numSolverJoints )
    {
        CD_WARNING("numRobotJoints(%d) != numSolverJoints(%d) !!!\n",numRobotJoints,numSolverJoints);
    }

    std::vector<double> qMin, qMax;
    for(unsigned int joint=0;joint<numRobotJoints;joint++)
    {
        double min, max;
        iControlLimits->getLimits(joint,&min,&max);
        qMin.push_back(min);
        qMax.push_back(max);
        CD_INFO("Joint %d limits: [%f,%f]\n",joint,min,max);
    }
    if( qMin[0] == qMax[0] )
    {
        CD_WARNING("Not setting joint limits on solver, because qMin[0] == qMax[0].\n");
    }
    else
    {
        iCartesianSolver->setLimits(qMin,qMax);
    }

    return this->start();
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::close()
{
    this->stop();
	robotDevice.close();
    solverDevice.close();
    return true;
}

// -----------------------------------------------------------------------------

int roboticslab::BasicCartesianControl::getCurrentState()
{
    int tmp;
    currentStateReady.wait();
    tmp = currentState;
    currentStateReady.post();

    return tmp;
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::setCurrentState(int value)
{
    currentStateReady.wait();
    currentState = value;
    currentStateReady.post();
}

// -----------------------------------------------------------------------------

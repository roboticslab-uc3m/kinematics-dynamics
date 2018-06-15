// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicTwoLimbCartesianControl.hpp"

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::BasicTwoLimbCartesianControl::open(yarp::os::Searchable& config)
{
    CD_DEBUG("Config: %s.\n", config.toString().c_str());

    yarp::os::Bottle limbA = config.findGroup("limbA");
    if( ! configureLimbA(limbA) )
        return false;

    yarp::os::Bottle limbB = config.findGroup("limbB");
    if( ! configureLimbB(limbB) )
        return false;

    return this->start();
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicTwoLimbCartesianControl::configureLimbA(yarp::os::Bottle& config)
{
    //-- Solver --

    //-- solver
    std::string solver = config.check("solver",yarp::os::Value(DEFAULT_SOLVER),"solver device type").asString();
    CD_INFO("solver: %s [%s]\n",solver.c_str(),DEFAULT_SOLVER);

    //-- kinematics
    std::string kinematics = config.check("kinematics",yarp::os::Value(DEFAULT_KINEMATICS_A),"limb kinematic description").asString();
    CD_INFO("kinematics: %s [%s]\n", kinematics.c_str(),DEFAULT_KINEMATICS_A);

    yarp::os::Property solverOptions;
    solverOptions.put("device",solver);
    solverOptions.put("kinematics",kinematics);

    solverDeviceA.open(solverOptions);
    if( ! solverDeviceA.isValid() )
    {
        CD_ERROR("solver device not valid: %s.\n",solver.c_str());
        return false;
    }
    if( ! solverDeviceA.view(iCartesianSolverA) )
    {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solver.c_str());
        return false;
    }

    //-- Robot --

    //-- remote
    std::string remote = config.check("remote",yarp::os::Value(DEFAULT_REMOTE_A),"remote robot").asString();
    CD_INFO("remote: %s [%s]\n",remote.c_str(),DEFAULT_REMOTE_A);

    yarp::os::Property robotOptions;
    robotOptions.fromString( config.toString() );
    robotOptions.put("device","remote_controlboard");
    std::string BasicTwoLimbCartesianControlStr("/BasicTwoLimbCartesianControl");
    robotOptions.put("local",BasicTwoLimbCartesianControlStr+remote);
    robotOptions.put("remote",remote);
    robotDeviceA.open(robotOptions);
    if( ! robotDeviceA.isValid() ) {
        CD_ERROR("robot remote not valid: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iEncodersA) ) {
        CD_ERROR("Could not view iEncoders in: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iPositionControlA) ) {
        CD_ERROR("Could not view iPositionControl in: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iVelocityControlA) ) {
        CD_ERROR("Could not view iVelocityControl in: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iControlLimitsA) ) {
        CD_ERROR("Could not view iControlLimits in: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iControlModeA) ) {
        CD_ERROR("Could not view iControlMode in: %s.\n",remote.c_str());
        return false;
    }

    iEncodersA->getAxes(&numRobotJointsA);
    CD_INFO("numRobotJoints: %d.\n",numRobotJointsA);

    iCartesianSolverA->getNumJoints( &numSolverJointsA );
    CD_INFO("numSolverJoints: %d.\n",numSolverJointsA);

    if( numRobotJointsA != numSolverJointsA )
    {
        CD_WARNING("numRobotJoints(%d) != numSolverJoints(%d) !!!\n",numRobotJointsA,numSolverJointsA);
    }

    std::vector<double> qMin, qMax;
    for(unsigned int joint=0;joint<numRobotJointsA;joint++)
    {
        double min, max;
        iControlLimitsA->getLimits(joint,&min,&max);
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
        iCartesianSolverA->setLimits(qMin,qMax);
    }

    double nonSingular[6]={0,0,-2,4,-2,0};
    iPositionControlA->positionMove(&nonSingular[0]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicTwoLimbCartesianControl::configureLimbB(yarp::os::Bottle& config)
{
    //-- Solver --

    //-- solver
    std::string solver = config.check("solver",yarp::os::Value(DEFAULT_SOLVER),"solver device type").asString();
    CD_INFO("solver: %s [%s]\n",solver.c_str(),DEFAULT_SOLVER);
    //-- kinematics
    std::string kinematics = config.check("kinematics",yarp::os::Value(DEFAULT_KINEMATICS_B),"limb kinematic description").asString();
    CD_INFO("kinematics: %s [%s]\n", kinematics.c_str(),DEFAULT_KINEMATICS_B);

    yarp::os::Property solverOptions;
    solverOptions.put("device",solver);
    solverOptions.put("kinematics",kinematics);

    solverDeviceB.open(solverOptions);
    if( ! solverDeviceB.isValid() )
    {
        CD_ERROR("solver device not valid: %s.\n",solver.c_str());
        return false;
    }
    if( ! solverDeviceB.view(iCartesianSolverB) )
    {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solver.c_str());
        return false;
    }

    //-- Robot --

    //-- remote
    std::string remote = config.check("remote",yarp::os::Value(DEFAULT_REMOTE_B),"remote robot").asString();
    CD_INFO("remote: %s [%s]\n",remote.c_str(),DEFAULT_REMOTE_B);

    yarp::os::Property robotOptions;
    robotOptions.fromString( config.toString() );
    robotOptions.put("device","remote_controlboard");
    std::string BasicTwoLimbCartesianControlStr("/BasicTwoLimbCartesianControl");
    robotOptions.put("local",BasicTwoLimbCartesianControlStr+remote);
    robotOptions.put("remote",remote);
    robotDeviceB.open(robotOptions);
    if( ! robotDeviceB.isValid() ) {
        CD_ERROR("robot remote not valid: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iEncodersB) ) {
        CD_ERROR("Could not view iEncoders in: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iPositionControlB) ) {
        CD_ERROR("Could not view iPositionControl in: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iVelocityControlB) ) {
        CD_ERROR("Could not view iVelocityControl in: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iControlLimitsB) ) {
        CD_ERROR("Could not view iControlLimits in: %s.\n",remote.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iControlModeB) ) {
        CD_ERROR("Could not view iControlMode in: %s.\n",remote.c_str());
        return false;
    }

    iEncodersB->getAxes(&numRobotJointsB);
    CD_INFO("numRobotJoints: %d.\n",numRobotJointsB);

    iCartesianSolverB->getNumJoints( &numSolverJointsB );
    CD_INFO("numSolverJoints: %d.\n",numSolverJointsB);

    if( numRobotJointsB != numSolverJointsB )
    {
        CD_WARNING("numRobotJoints(%d) != numSolverJoints(%d) !!!\n",numRobotJointsB,numSolverJointsB);
    }

    std::vector<double> qMin, qMax;
    for(unsigned int joint=0;joint<numRobotJointsB;joint++)
    {
        double min, max;
        iControlLimitsB->getLimits(joint,&min,&max);
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
        iCartesianSolverB->setLimits(qMin,qMax);
    }

    double nonSingular[6]={0,0,-2,4,-2,0};
    iPositionControlB->positionMove(&nonSingular[0]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicTwoLimbCartesianControl::close()
{
    robotDeviceA.close();
    robotDeviceB.close();

    solverDeviceA.close();
    solverDeviceB.close();

    return true;
}

// -----------------------------------------------------------------------------

int roboticslab::BasicTwoLimbCartesianControl::getCurrentState()
{
    int tmp;
    currentStateReady.wait();
    tmp = currentState;
    currentStateReady.post();

    return tmp;
}

// -----------------------------------------------------------------------------

void roboticslab::BasicTwoLimbCartesianControl::setCurrentState(int value)
{
    currentStateReady.wait();
    currentState = value;
    currentStateReady.post();
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicTwoLimbCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::BasicTwoLimbCartesianControl::open(yarp::os::Searchable& config) {

    CD_DEBUG("BasicTwoLimbCartesianControl config: %s.\n", config.toString().c_str());

    std::string solverStr = config.check("solver",yarp::os::Value(DEFAULT_SOLVER),"cartesian solver").asString();
    std::string limbA = config.check("limbA",yarp::os::Value(DEFAULT_LIMB_A),"limb A").asString();
    std::string limbB = config.check("limbB",yarp::os::Value(DEFAULT_LIMB_B),"limb B").asString();

    std::string BasicTwoLimbCartesianControlStr("/BasicTwoLimbCartesianControl");

    //--

    yarp::os::Property solverOptionsA;
    solverOptionsA.fromString( config.toString() );
    solverOptionsA.put("device",solverStr);

    solverDeviceA.open(solverOptionsA);
    if( ! solverDeviceA.isValid() ) {
        CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        return false;
    }
    if( ! solverDeviceA.view(iCartesianSolverA) ) {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;
    }

    yarp::os::Property robotOptionsA;
    robotOptionsA.fromString( config.toString() );
    robotOptionsA.put("device","remote_controlboard");
    robotOptionsA.put("local",BasicTwoLimbCartesianControlStr+limbA);
    robotOptionsA.put("remote",limbA);
    robotDeviceA.open(robotOptionsA);
    if( ! robotDeviceA.isValid() ) {
        CD_ERROR("robot device not valid: %s.\n",limbA.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iEncodersA) ) {
        CD_ERROR("Could not view iEncoders in: %s.\n",limbA.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iPositionControlA) ) {
        CD_ERROR("Could not view iPositionControl in: %s.\n",limbA.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iVelocityControlA) ) {
        CD_ERROR("Could not view iVelocityControl in: %s.\n",limbA.c_str());
        return false;
    }
    if( ! robotDeviceA.view(iControlLimitsA) ) {
        CD_ERROR("Could not view iControlLimits in: %s.\n",limbA.c_str());
        return false;
    }

    iEncodersA->getAxes(&numRobotJointsA);
    CD_INFO("numRobotJoints: %d.\n",numRobotJointsA);

    iCartesianSolverA->getNumLinks( &numSolverLinksA );
    CD_INFO("numSolverLinks: %d.\n",numSolverLinksA);

    if( numRobotJointsA != numSolverLinksA )
    {
        CD_WARNING("numRobotJoints(%d) != numSolverLinks(%d) !!!\n",numRobotJointsA,numSolverLinksA);
    }

    std::vector<double> qMinA, qMaxA;
    for(unsigned int joint=0;joint<numRobotJointsA;joint++)
    {
        double min, max;
        iControlLimitsA->getLimits(joint,&min,&max);
        qMinA.push_back(min);
        qMaxA.push_back(max);
        CD_INFO("Joint %d limits: [%f,%f]\n",joint,min,max);
    }
    if( qMinA[0] == qMaxA[0] )
    {
        CD_WARNING("Not setting joint limits on solver, because qMin[0] == qMax[0].\n");
    }
    else
    {
        iCartesianSolverA->setLimits(qMinA,qMaxA);
    }

    //--

    yarp::os::Property solverOptionsB;
    solverOptionsB.fromString( config.toString() );
    solverOptionsB.put("device",solverStr);

    solverDeviceB.open(solverOptionsB);
    if( ! solverDeviceB.isValid() ) {
        CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        return false;
    }
    if( ! solverDeviceB.view(iCartesianSolverB) ) {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;
    }

    yarp::os::Property robotOptionsB;
    robotOptionsB.fromString( config.toString() );
    robotOptionsB.put("device","remote_controlboard");
    robotOptionsB.put("local",BasicTwoLimbCartesianControlStr+limbB);
    robotOptionsB.put("remote",limbB);
    robotDeviceB.open(robotOptionsB);
    if( ! robotDeviceB.isValid() ) {
        CD_ERROR("robot device not valid: %s.\n",limbA.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iEncodersB) ) {
        CD_ERROR("Could not view iEncoders in: %s.\n",limbA.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iPositionControlB) ) {
        CD_ERROR("Could not view iPositionControl in: %s.\n",limbA.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iVelocityControlB) ) {
        CD_ERROR("Could not view iVelocityControl in: %s.\n",limbA.c_str());
        return false;
    }
    if( ! robotDeviceB.view(iControlLimitsB) ) {
        CD_ERROR("Could not view iControlLimits in: %s.\n",limbA.c_str());
        return false;
    }

    iEncodersB->getAxes(&numRobotJointsB);
    CD_INFO("numRobotJointsB: %d.\n",numRobotJointsB);

    iCartesianSolverB->getNumLinks( &numSolverLinksB );
    CD_INFO("numSolverLinksB: %d.\n",numSolverLinksB);

    if( numRobotJointsB != numSolverLinksB )
    {
        CD_WARNING("numRobotJointsB(%d) != numSolverLinksB(%d) !!!\n",numRobotJointsB,numSolverLinksB);
    }

    std::vector<double> qMinB, qMaxB;
    for(unsigned int joint=0;joint<numRobotJointsB;joint++)
    {
        double min, max;
        iControlLimitsB->getLimits(joint,&min,&max);
        qMinB.push_back(min);
        qMaxB.push_back(max);
        CD_INFO("Joint %d limits: [%f,%f]\n",joint,min,max);
    }
    if( qMinB[0] == qMaxB[0] )
    {
        CD_WARNING("Not setting joint limits on solver, because qMin[0] == qMax[0].\n");
    }
    else
    {
        iCartesianSolverB->setLimits(qMinB,qMaxB);
    }


    return this->start();
}

// -----------------------------------------------------------------------------

bool teo::BasicTwoLimbCartesianControl::close() {

    return true;
}

// -----------------------------------------------------------------------------

int teo::BasicTwoLimbCartesianControl::getCurrentState()
{
    int tmp;
    currentStateReady.wait();
    tmp = currentState;
    currentStateReady.post();

    return tmp;
}

// -----------------------------------------------------------------------------

void teo::BasicTwoLimbCartesianControl::setCurrentState(int value)
{
    currentStateReady.wait();
    currentState = value;
    currentStateReady.post();
}

// -----------------------------------------------------------------------------

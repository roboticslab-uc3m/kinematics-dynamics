// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicTwoLimbCartesianControl.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::BasicTwoLimbCartesianControl::open(yarp::os::Searchable& config)
{

    CD_DEBUG("BasicTwoLimbCartesianControl config: %s.\n", config.toString().c_str());

    yarp::os::Bottle limbA = config.findGroup("limbA");
    if( ! configureLimbA(limbA) )
        return false;

    /*yarp::os::Bottle limbB = config.findGroup("limbB");
    if( ! configureLimbB(limbB) )
        return false;*/

    return this->start();
}

// -----------------------------------------------------------------------------

bool teo::BasicTwoLimbCartesianControl::configureLimbA(yarp::os::Bottle& config)
{
    std::string solver = config.check("solver",yarp::os::Value(DEFAULT_SOLVER),"solver device type").asString();
    //std::string angleRepr = config.check("angleRepr",yarp::os::Value(DEFAULT_ANG_REPR),"angle representation").asString();
    std::string remote = config.check("remote",yarp::os::Value(DEFAULT_REMOTE_A),"remote robot").asString();
    //std::string kinematics = config.check("kinematics",yarp::os::Value(DEFAULT_KINEMATICS_A),"limb kinematic description").asString();

    CD_INFO("solver: %s [%s]\n",solver.c_str(),DEFAULT_SOLVER);
    //...
    CD_INFO("remote: %s [%s]\n",remote.c_str(),DEFAULT_REMOTE_A);
    //printf("\t--limbA kinematics %s [%s]\n",DEFAULT_KINEMATICS_A);

    //-- Solver --
    yarp::os::Property solverOptions;
    solverOptions.fromString( config.toString() );
    solverOptions.put("device",solver);
    //solverOptions.put("kinamatics",kinematics);

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

    return true;
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

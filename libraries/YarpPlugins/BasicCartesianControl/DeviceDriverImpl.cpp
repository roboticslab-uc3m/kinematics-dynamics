// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::BasicCartesianControl::open(yarp::os::Searchable& config)
{
    CD_DEBUG("BasicCartesianControl config: %s.\n", config.toString().c_str());

    gain = config.check("controllerGain",yarp::os::Value(DEFAULT_GAIN),"controller gain").asDouble();
    maxJointVelocity = config.check("maxJointVelocity",yarp::os::Value(DEFAULT_QDOT_LIMIT),"maximum joint velocity (meters/second or degrees/second)").asDouble();
    duration = config.check("trajectoryDuration",yarp::os::Value(DEFAULT_DURATION),"trajectory duration (seconds)").asDouble();
    cmcRateMs = config.check("cmcRateMs",yarp::os::Value(DEFAULT_CMC_RATE_MS),"CMC rate (milliseconds)").asInt();
    waitPeriodMs = config.check("waitPeriodMs",yarp::os::Value(DEFAULT_WAIT_PERIOD_MS),"wait command period (milliseconds)").asInt();

    std::string referenceFrameStr = config.check("referenceFrame", yarp::os::Value(DEFAULT_REFERENCE_FRAME),
                "reference frame (base|tcp)").asString();

    if (referenceFrameStr == "base")
    {
        referenceFrame = ICartesianSolver::BASE_FRAME;
    }
    else if (referenceFrameStr == "tcp")
    {
        referenceFrame = ICartesianSolver::TCP_FRAME;
    }
    else
    {
        CD_ERROR("Unsupported reference frame: %s.\n", referenceFrameStr.c_str());
        return false;
    }

    std::string robotStr = config.check("robot",yarp::os::Value(DEFAULT_ROBOT),"robot device").asString();
    std::string solverStr = config.check("solver",yarp::os::Value(DEFAULT_SOLVER),"cartesian solver device").asString();

    yarp::os::Property robotOptions;
    robotOptions.fromString( config.toString() );
    robotOptions.put("device",robotStr);
    robotOptions.setMonitor(config.getMonitor(), robotStr.c_str());
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

    qMin.resize(numRobotJoints);
    qMax.resize(numRobotJoints);

    yarp::os::Bottle bMin, bMax;

    for(unsigned int joint=0;joint<numRobotJoints;joint++)
    {
        double min, max;
        iControlLimits->getLimits(joint,&min,&max);
        qMin[joint] = min;
        qMax[joint] = max;
        bMin.addDouble(min);
        bMax.addDouble(max);
        CD_INFO("Joint %d limits: [%f,%f]\n",joint,min,max);
    }

    yarp::os::Property solverOptions;
    solverOptions.fromString( config.toString() );
    solverOptions.put("device",solverStr);
    solverOptions.put("mins", yarp::os::Value::makeList(bMin.toString().c_str()));
    solverOptions.put("maxs", yarp::os::Value::makeList(bMax.toString().c_str()));
    solverOptions.setMonitor(config.getMonitor(), solverStr.c_str());

    solverDevice.open(solverOptions);
    if( ! solverDevice.isValid() ) {
        CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        return false;
    }
    if( ! solverDevice.view(iCartesianSolver) ) {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;
    }

    iCartesianSolver->getNumJoints( &numSolverJoints );
    CD_INFO("numSolverJoints: %d.\n",numSolverJoints);

    if( numRobotJoints != numSolverJoints )
    {
        CD_WARNING("numRobotJoints(%d) != numSolverJoints(%d) !!!\n",numRobotJoints,numSolverJoints);
    }

    if( cmcRateMs != DEFAULT_CMC_RATE_MS )
    {
        yarp::os::RateThread::setRate(cmcRateMs);
    }

    return yarp::os::RateThread::start();
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::close()
{
    stopControl();
    yarp::os::RateThread::stop();
    robotDevice.close();
    solverDevice.close();
    return true;
}

// -----------------------------------------------------------------------------

int roboticslab::BasicCartesianControl::getCurrentState() const
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

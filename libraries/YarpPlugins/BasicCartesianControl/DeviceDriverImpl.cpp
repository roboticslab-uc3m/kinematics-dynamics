// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::BasicCartesianControl::open(yarp::os::Searchable& config)
{
    CD_DEBUG("BasicCartesianControl config: %s.\n", config.toString().c_str());

    gain = config.check("controllerGain", yarp::os::Value(DEFAULT_GAIN),
            "controller gain").asFloat64();

    duration = config.check("trajectoryDuration", yarp::os::Value(DEFAULT_DURATION),
            "trajectory duration (seconds)").asFloat64();

    cmcPeriodMs = config.check("cmcPeriodMs", yarp::os::Value(DEFAULT_CMC_PERIOD_MS),
            "CMC rate (milliseconds)").asInt32();

    waitPeriodMs = config.check("waitPeriodMs", yarp::os::Value(DEFAULT_WAIT_PERIOD_MS),
            "wait command period (milliseconds)").asInt32();

    streamingPeriodMs = config.check("streamingPeriodMs", yarp::os::Value(DEFAULT_STREAMING_PERIOD_MS),
            "streaming command send period (milliseconds)").asInt32();

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

    std::string robotStr = config.check("robot", yarp::os::Value(DEFAULT_ROBOT),
            "robot device").asString();

    std::string solverStr = config.check("solver", yarp::os::Value(DEFAULT_SOLVER),
            "cartesian solver device").asString();

    yarp::os::Property robotOptions;
    robotOptions.fromString(config.toString());
    robotOptions.put("device", robotStr);
    robotOptions.setMonitor(config.getMonitor(), robotStr.c_str());

    if (!robotDevice.open(robotOptions))
    {
        CD_ERROR("robot device not valid: %s.\n", robotStr.c_str());
        return false;
    }

    if (!robotDevice.view(iEncoders))
    {
        CD_ERROR("Could not view iEncoders in: %s.\n", robotStr.c_str());
        return false;
    }

    if (!robotDevice.view(iPositionControl))
    {
        CD_ERROR("Could not view iPositionControl in: %s.\n", robotStr.c_str());
        return false;
    }

    if (!robotDevice.view(iPositionDirect))
    {
        CD_ERROR("Could not view iPositionDirect in: %s.\n", robotStr.c_str());
        return false;
    }

    if (!robotDevice.view(iVelocityControl))
    {
        CD_ERROR("Could not view iVelocityControl in: %s.\n", robotStr.c_str());
        return false;
    }

    if (!robotDevice.view(iControlLimits))
    {
        CD_ERROR("Could not view iControlLimits in: %s.\n", robotStr.c_str());
        return false;
    }

    if (!robotDevice.view(iTorqueControl))
    {
        CD_ERROR("Could not view iTorqueControl in: %s.\n", robotStr.c_str());
        return false;
    }

    if (!robotDevice.view(iControlMode))
    {
        CD_ERROR("Could not view iControlMode in: %s.\n", robotStr.c_str());
        return false;
    }

    if (!robotDevice.view(iPreciselyTimed))
    {
        CD_WARNING("Could not view iPreciselyTimed in: %s. Using local timestamps.\n", robotStr.c_str());
    }

    iEncoders->getAxes(&numRobotJoints);
    CD_INFO("numRobotJoints: %d.\n", numRobotJoints);

    qRefSpeeds.resize(numRobotJoints);

    if (!iPositionControl->getRefSpeeds(qRefSpeeds.data()))
    {
        CD_ERROR("Could not retrieve reference speeds.\n");
        return false;
    }

    qMin.resize(numRobotJoints);
    qMax.resize(numRobotJoints);

    qdotMin.resize(numRobotJoints);
    qdotMax.resize(numRobotJoints);

    yarp::os::Bottle bMin, bMax;

    for (int joint = 0; joint < numRobotJoints; joint++)
    {
        double _qMin, _qMax;

        if (!iControlLimits->getLimits(joint, &_qMin, &_qMax))
        {
            CD_ERROR("Unable to retrieve position limits for joint %d.\n");
            return false;
        }

        qMin[joint] = _qMin;
        qMax[joint] = _qMax;

        double _qdotMin, _qdotMax;

        if (!iControlLimits->getVelLimits(joint, &_qdotMin, &_qdotMax))
        {
            CD_ERROR("Unable to retrieve speed limits for joint %d.\n");
            return false;
        }

        qdotMin[joint] = _qdotMin;
        qdotMax[joint] = _qdotMax;

        CD_INFO("Joint %d limits: [%f,%f] [%f,%f]\n", joint, _qMin, _qMax, _qdotMin, _qdotMax);

        bMin.addFloat64(_qMin);
        bMax.addFloat64(_qMax);
    }

    yarp::os::Property solverOptions;
    solverOptions.fromString(config.toString());
    solverOptions.put("device", solverStr);
    solverOptions.put("mins", yarp::os::Value::makeList(bMin.toString().c_str()));
    solverOptions.put("maxs", yarp::os::Value::makeList(bMax.toString().c_str()));
    solverOptions.setMonitor(config.getMonitor(), solverStr.c_str());

    if (!solverDevice.open(solverOptions))
    {
        CD_ERROR("solver device not valid: %s.\n", solverStr.c_str());
        return false;
    }

    if (!solverDevice.view(iCartesianSolver))
    {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n", solverStr.c_str());
        return false;
    }

    iCartesianSolver->getNumJoints(&numSolverJoints);
    CD_INFO("numSolverJoints: %d.\n", numSolverJoints);

    if (numRobotJoints != numSolverJoints)
    {
        CD_WARNING("numRobotJoints(%d) != numSolverJoints(%d) !!!\n", numRobotJoints, numSolverJoints);
    }

    if (cmcPeriodMs != DEFAULT_CMC_PERIOD_MS)
    {
        yarp::os::PeriodicThread::setPeriod(cmcPeriodMs * 0.001);
    }

    return yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::close()
{
    stopControl();
    yarp::os::PeriodicThread::stop();
    robotDevice.close();
    solverDevice.close();
    return true;
}

// -----------------------------------------------------------------------------

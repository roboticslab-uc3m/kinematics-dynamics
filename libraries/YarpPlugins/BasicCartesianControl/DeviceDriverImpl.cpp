// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <yarp/os/LogStream.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::BasicCartesianControl::open(yarp::os::Searchable& config)
{
    yDebug() << "BasicCartesianControl config:" << config.toString();

    gain = config.check("controllerGain", yarp::os::Value(DEFAULT_GAIN),
            "controller gain").asFloat64();

    duration = config.check("trajectoryDuration", yarp::os::Value(DEFAULT_DURATION),
            "trajectory duration (seconds)").asFloat64();

    cmcPeriodMs = config.check("cmcPeriodMs", yarp::os::Value(DEFAULT_CMC_PERIOD_MS),
            "CMC rate (milliseconds)").asInt32();

    waitPeriodMs = config.check("waitPeriodMs", yarp::os::Value(DEFAULT_WAIT_PERIOD_MS),
            "wait command period (milliseconds)").asInt32();

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
        yError() << "Unsupported reference frame:" << referenceFrameStr;
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
        yError() << "Tobot device not valid:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iEncoders))
    {
        yError() << "Could not view iEncoders in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iPositionControl))
    {
        yError() << "Could not view iPositionControl in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iPositionDirect))
    {
        yError() << "Could not view iPositionDirect in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iVelocityControl))
    {
        yError() << "Could not view iVelocityControl in:" << robotStr;
        return false;
    }



    if (!robotDevice.view(iTorqueControl))
    {
        yError() << "Could not view iTorqueControl in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iControlMode))
    {
        yError() << "Could not view iControlMode in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iPreciselyTimed))
    {
        yWarning("Could not view iPreciselyTimed in: %s, using local timestamps", robotStr.c_str());
    }

    iEncoders->getAxes(&numRobotJoints);
    yInfo() << "numRobotJoints:" << numRobotJoints;

    qRefSpeeds.resize(numRobotJoints);

    if (!iPositionControl->getRefSpeeds(qRefSpeeds.data()))
    {
        yError() << "Could not retrieve reference speeds";
        return false;
    }

    yarp::os::Property solverOptions;
    solverOptions.fromString(config.toString());
    solverOptions.put("device", solverStr);

    if (!config.check("mins") || !config.check("maxs") || !config.check("maxvels"))
    {
        yInfo() << "Using joint limits provided by robot";

        yarp::dev::IControlLimits * iControlLimits;

        if (!robotDevice.view(iControlLimits))
        {
            yError() << "Could not view iControlLimits in:" << robotStr;
            return false;
        }

        qMin.resize(numRobotJoints);
        qMax.resize(numRobotJoints);

        qdotMin.resize(numRobotJoints);
        qdotMax.resize(numRobotJoints);

        yarp::os::Bottle bMin, bMax, bMaxVel;

        for (int joint = 0; joint < numRobotJoints; joint++)
        {
            double _qMin, _qMax;

            if (!iControlLimits->getLimits(joint, &_qMin, &_qMax))
            {
                yError() << "Unable to retrieve position limits for joint" << joint;
                return false;
            }

            qMin[joint] = _qMin;
            qMax[joint] = _qMax;

            double _qdotMin, _qdotMax;

            if (!iControlLimits->getVelLimits(joint, &_qdotMin, &_qdotMax))
            {
                yError() << "Unable to retrieve speed limits for joint" << joint;
                return false;
            }

            qdotMin[joint] = _qdotMin;
            qdotMax[joint] = _qdotMax;

            yInfo("Joint %d limits: [%f,%f] [%f,%f]", joint, _qMin, _qMax, _qdotMin, _qdotMax);

            bMin.addFloat64(_qMin);
            bMax.addFloat64(_qMax);
            bMaxVel.addFloat64(_qdotMax);
        }

        solverOptions.put("mins", yarp::os::Value::makeList(bMin.toString().c_str()));
        solverOptions.put("maxs", yarp::os::Value::makeList(bMax.toString().c_str()));
        solverOptions.put("maxvels", yarp::os::Value::makeList(bMaxVel.toString().c_str()));
    }
    else
    {
        yInfo() << "Using joint limits provided via user configuration";
    }

    solverOptions.setMonitor(config.getMonitor(), solverStr.c_str());

    if (!solverDevice.open(solverOptions))
    {
        yError() << "Solver device not valid:" << solverStr;
        return false;
    }

    if (!solverDevice.view(iCartesianSolver))
    {
        yError() << "Could not view iCartesianSolver in:" << solverStr;
        return false;
    }

    numSolverJoints = iCartesianSolver->getNumJoints();
    yInfo() << "numSolverJoints:" << numSolverJoints;

    if (numRobotJoints != numSolverJoints)
    {
        yWarning("numRobotJoints(%d) != numSolverJoints(%d)", numRobotJoints, numSolverJoints);
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

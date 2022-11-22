// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_SOLVER = "KdlSolver";
constexpr auto DEFAULT_ROBOT = "remote_controlboard";
constexpr auto DEFAULT_GAIN = 0.05;
constexpr auto DEFAULT_DURATION = 10.0;
constexpr auto DEFAULT_CMC_PERIOD_MS = 50;
constexpr auto DEFAULT_WAIT_PERIOD_MS = 30;
constexpr auto DEFAULT_REFERENCE_FRAME = "base";

// ------------------- DeviceDriver Related ------------------------------------

bool BasicCartesianControl::open(yarp::os::Searchable& config)
{
    gain = config.check("controllerGain", yarp::os::Value(DEFAULT_GAIN),
            "controller gain").asFloat64();

    duration = config.check("trajectoryDuration", yarp::os::Value(DEFAULT_DURATION),
            "trajectory duration (seconds)").asFloat64();

    cmcPeriodMs = config.check("cmcPeriodMs", yarp::os::Value(DEFAULT_CMC_PERIOD_MS),
            "CMC rate (milliseconds)").asInt32();

    waitPeriodMs = config.check("waitPeriodMs", yarp::os::Value(DEFAULT_WAIT_PERIOD_MS),
            "wait command period (milliseconds)").asInt32();

    usePosdMovl = config.check("usePosdMovl", "execute MOVL commands in POSD mode using IK");
    enableFailFast = config.check("enableFailFast", "enable fail-fast mode for MOVL commands");

    if (enableFailFast && !usePosdMovl)
    {
        yCError(BCC) << "Cannot use --enableFailFast without --usePosdMovl";
        return false;
    }

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
        yCError(BCC) << "Unsupported reference frame:" << referenceFrameStr;
        return false;
    }

    auto robotStr = config.check("robot", yarp::os::Value(DEFAULT_ROBOT), "robot device").asString();
    auto solverStr = config.check("solver", yarp::os::Value(DEFAULT_SOLVER), "cartesian solver device").asString();

    yarp::os::Property robotOptions;
    robotOptions.fromString(config.toString());
    robotOptions.put("device", robotStr);
    robotOptions.setMonitor(config.getMonitor(), robotStr.c_str());

    if (!robotDevice.open(robotOptions))
    {
        yCError(BCC) << "Robot device not valid:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iEncoders))
    {
        yCError(BCC) << "Could not view iEncoders in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iPositionControl))
    {
        yCError(BCC) << "Could not view iPositionControl in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iPositionDirect))
    {
        yCError(BCC) << "Could not view iPositionDirect in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iVelocityControl))
    {
        yCError(BCC) << "Could not view iVelocityControl in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iTorqueControl))
    {
        yCError(BCC) << "Could not view iTorqueControl in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iControlMode))
    {
        yCError(BCC) << "Could not view iControlMode in:" << robotStr;
        return false;
    }

    if (!robotDevice.view(iPreciselyTimed))
    {
        yCWarning(BCC, "Could not view iPreciselyTimed in: %s, using local timestamps", robotStr.c_str());
    }

    iEncoders->getAxes(&numJoints);
    yCInfo(BCC) << "Number of robot joints:" << numJoints;

    qRefSpeeds.resize(numJoints);

    if (!iPositionControl->getRefSpeeds(qRefSpeeds.data()))
    {
        yCError(BCC) << "Could not retrieve reference speeds";
        return false;
    }

    yarp::os::Property solverOptions;
    solverOptions.fromString(config.toString());
    solverOptions.put("device", solverStr);

    if (!config.check("mins") || !config.check("maxs") || !config.check("maxvels"))
    {
        yCInfo(BCC) << "Using joint limits provided by robot";

        yarp::dev::IControlLimits * iControlLimits;

        if (!robotDevice.view(iControlLimits))
        {
            yCError(BCC) << "Could not view iControlLimits in:" << robotStr;
            return false;
        }

        qMin.resize(numJoints);
        qMax.resize(numJoints);

        qdotMin.resize(numJoints);
        qdotMax.resize(numJoints);

        yarp::os::Bottle bMin, bMax, bMaxVel;

        for (int joint = 0; joint < numJoints; joint++)
        {
            double _qMin, _qMax;

            if (!iControlLimits->getLimits(joint, &_qMin, &_qMax))
            {
                yCError(BCC) << "Unable to retrieve position limits for joint" << joint;
                return false;
            }

            qMin[joint] = _qMin;
            qMax[joint] = _qMax;

            double _qdotMin, _qdotMax;

            if (!iControlLimits->getVelLimits(joint, &_qdotMin, &_qdotMax))
            {
                yCError(BCC) << "Unable to retrieve speed limits for joint" << joint;
                return false;
            }

            qdotMin[joint] = _qdotMin;
            qdotMax[joint] = _qdotMax;

            yCInfo(BCC, "Joint %d limits: [%f,%f] [%f,%f]", joint, _qMin, _qMax, _qdotMin, _qdotMax);

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
        yCInfo(BCC) << "Using joint limits provided via user configuration";
    }

    solverOptions.setMonitor(config.getMonitor(), solverStr.c_str());

    if (!solverDevice.open(solverOptions))
    {
        yCError(BCC) << "Solver device not valid:" << solverStr;
        return false;
    }

    if (!solverDevice.view(iCartesianSolver))
    {
        yCError(BCC) << "Could not view iCartesianSolver in:" << solverStr;
        return false;
    }

    int numSolverJoints = iCartesianSolver->getNumJoints();

    if (numSolverJoints != numJoints)
    {
        yCError(BCC, "numSolverJoints(%d) != numRobotJoints(%d)", numSolverJoints, numJoints);
        return false;
    }

    yCInfo(BCC) << "Number of solver TCPs:" << iCartesianSolver->getNumTcps();

    yarp::os::PeriodicThread::setPeriod(cmcPeriodMs * 0.001);

    currentState = VOCAB_CC_NOT_CONTROLLING;
    streamingCommand = VOCAB_CC_NOT_SET;
    movementStartTime = 0.0;
    cmcSuccess = true;

    return yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool BasicCartesianControl::close()
{
    stopControl();
    yarp::os::PeriodicThread::stop();
    robotDevice.close();
    solverDevice.close();
    return true;
}

// -----------------------------------------------------------------------------

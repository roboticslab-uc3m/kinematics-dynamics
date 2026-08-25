// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool BasicCartesianControl::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(BCC) << "Invalid parameters";
        return false;
    }

    if (m_enableFailFast && !m_usePosdMovel)
    {
        yCError(BCC) << "Cannot use --enableFailFast without --usePosdMovel";
        return false;
    }

    if (m_referenceFrame == "base")
    {
        referenceFrame = ICartesianSolver::Frame::BASE;
    }
    else if (m_referenceFrame == "tcp")
    {
        referenceFrame = ICartesianSolver::Frame::TCP;
    }
    else
    {
        yCError(BCC) << "Unsupported reference frame:" << m_referenceFrame;
        return false;
    }

    if (m_controllerGain < 0.0)
    {
        yCError(BCC) << "Controller gain must be positive";
        return false;
    }

    if (m_trajectoryDuration < 0.0)
    {
        yCError(BCC) << "Trajectory duration must be positive or zero";
        return false;
    }
    else if (m_trajectoryDuration == 0.0)
    {
        yCInfo(BCC) << "Duration set to zero, therefore trajectory execution time will depend on reference speed and acceleration";
    }
    else
    {
        yCInfo(BCC) << "Trajectory duration forced to" << m_trajectoryDuration << "seconds regardless of velocity profile";
    }

    if (m_trajectoryRefSpeed <= 0.0)
    {
        yCError(BCC) << "Trajectory reference speed must be positive";
        return false;
    }

    if (m_trajectoryRefAccel <= 0.0)
    {
        yCError(BCC) << "Trajectory reference acceleration must be positive";
        return false;
    }

    if (m_cmcPeriodMs <= 0)
    {
        yCError(BCC) << "CMC period must be positive";
        return false;
    }

    if (m_waitPeriodMs <= 0)
    {
        yCError(BCC) << "Wait period must be positive";
        return false;
    }

    yarp::os::Property robotOptions;
    robotOptions.fromString(config.toString());
    robotOptions.put("device", m_robot);

    if (!robotDevice.open(robotOptions))
    {
        yCError(BCC) << "Robot device not valid:" << m_robot;
        return false;
    }

    if (!robotDevice.view(iEncoders))
    {
        yCError(BCC) << "Could not view iEncoders in:" << m_robot;
        return false;
    }

    if (!robotDevice.view(iPositionControl))
    {
        yCError(BCC) << "Could not view iPositionControl in:" << m_robot;
        return false;
    }

    if (!robotDevice.view(iPositionDirect))
    {
        yCError(BCC) << "Could not view iPositionDirect in:" << m_robot;
        return false;
    }

    if (!robotDevice.view(iVelocityControl))
    {
        yCError(BCC) << "Could not view iVelocityControl in:" << m_robot;
        return false;
    }

    if (!robotDevice.view(iTorqueControl))
    {
        yCError(BCC) << "Could not view iTorqueControl in:" << m_robot;
        return false;
    }

    if (!robotDevice.view(iControlMode))
    {
        yCError(BCC) << "Could not view iControlMode in:" << m_robot;
        return false;
    }

    if (!robotDevice.view(iPreciselyTimed))
    {
        yCWarning(BCC, "Could not view iPreciselyTimed in: %s, using local timestamps", m_robot.c_str());
    }

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    iEncoders->getAxes(numJoints);
#else
    iEncoders->getAxes(&numJoints);
#endif
    yCInfo(BCC) << "Number of robot joints:" << numJoints;

    qRefSpeeds.resize(numJoints);

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    if (!iPositionControl->getTrajSpeeds(qRefSpeeds.data()))
#else
    if (!iPositionControl->getRefSpeeds(qRefSpeeds.data()))
#endif
    {
        yCError(BCC) << "Could not retrieve reference speeds";
        return false;
    }

    yarp::os::Property solverOptions;
    solverOptions.fromString(config.toString());
    solverOptions.put("device", m_solver);

    {
        yCInfo(BCC) << "Using joint limits provided by robot";

        yarp::dev::IControlLimits * iControlLimits;

        if (!robotDevice.view(iControlLimits))
        {
            yCError(BCC) << "Could not view iControlLimits in:" << m_robot;
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

#if YARP_VERSION_COMPARE(>=, 4,0,0)
            if (!iControlLimits->getPosLimits(joint, &_qMin, &_qMax))
#else
            if (!iControlLimits->getLimits(joint, &_qMin, &_qMax))
#endif
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

    if (!solverDevice.open(solverOptions))
    {
        yCError(BCC) << "Solver device not valid:" << m_solver;
        return false;
    }

    if (!solverDevice.view(iCartesianSolver))
    {
        yCError(BCC) << "Could not view iCartesianSolver in:" << m_solver;
        return false;
    }

    if (std::size_t numSolverJoints; iCartesianSolver->getNumJoints(numSolverJoints) && numSolverJoints != numJoints)
    {
        yCError(BCC, "numSolverJoints(%zu) != numRobotJoints(%zu)", numSolverJoints, numJoints);
        return false;
    }

    std::size_t numSolverTcps;
    iCartesianSolver->getNumTcps(numSolverTcps);

    yCInfo(BCC) << "Number of solver TCPs:" << numSolverTcps;

    return yarp::os::PeriodicThread::setPeriod(m_cmcPeriodMs * 0.001) && yarp::os::PeriodicThread::start();
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

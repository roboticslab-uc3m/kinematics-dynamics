// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

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

    if (m_enableFailFast && !m_usePosdMovl)
    {
        yCError(BCC) << "Cannot use --enableFailFast without --usePosdMovl";
        return false;
    }

    if (m_referenceFrame == "base")
    {
        referenceFrame = ICartesianSolver::BASE_FRAME;
    }
    else if (m_referenceFrame == "tcp")
    {
        referenceFrame = ICartesianSolver::TCP_FRAME;
    }
    else
    {
        yCError(BCC) << "Unsupported reference frame:" << m_referenceFrame;
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

    if (int numSolverJoints = iCartesianSolver->getNumJoints(); numSolverJoints != numJoints)
    {
        yCError(BCC, "numSolverJoints(%d) != numRobotJoints(%d)", numSolverJoints, numJoints);
        return false;
    }

    yCInfo(BCC) << "Number of solver TCPs:" << iCartesianSolver->getNumTcps();

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

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath>  //-- std::abs
#include <algorithm>
#include <functional>
#include <vector>

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

#include "KdlTrajectory.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::BasicCartesianControl::stat(int &state, std::vector<double> &x)
{
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->fwdKin(currentQ,x) )
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }
    state = getCurrentState();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->invKin(xd,currentQ,q,referenceFrame) )
    {
        CD_ERROR("invKin failed.\n");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::movj(const std::vector<double> &xd)
{
    std::vector<double> currentQ(numRobotJoints), qd;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->invKin(xd,currentQ,qd,referenceFrame) )
    {
        CD_ERROR("invKin failed.\n");
        return false;
    }

    //-- Find out the maximum time to move
    double max_time = 0;
    for(unsigned int joint=0;joint<numSolverJoints;joint++)
    {
        CD_INFO("dist[%d]: %f\n",joint,std::abs(qd[joint]-currentQ[joint]));
        if (std::abs((qd[joint]-currentQ[joint]) / maxJointVelocity) > max_time)
        {
            max_time = std::abs( (qd[joint]-currentQ[joint]) / maxJointVelocity);
            CD_INFO(" -->candidate: %f\n",max_time);
        }
    }
    CD_INFO("max_time[final]: %f\n",max_time);

    //-- Compute, store old and set joint velocities given this time
    std::vector<double> vmo;
    for(unsigned int joint=0;joint<numRobotJoints;joint++)
    {
        if( joint >= numSolverJoints )
        {
            vmo.push_back( 0.0 );
            CD_INFO("vmo[%d]: 0.0 (forced)\n",joint);
        }
        else
        {
            vmo.push_back( std::abs(qd[joint] - currentQ[joint])/max_time );
            CD_INFO("vmo[%d]: %f\n",joint,vmo[joint]);
        }
    }
    vmoStored.resize(numRobotJoints);
    if ( ! iPositionControl->getRefSpeeds( vmoStored.data() ) )
    {
         CD_ERROR("getRefSpeeds (for storing) failed.\n");
         return false;
    }
    if ( ! iPositionControl->setRefSpeeds( vmo.data() ) )
    {
         CD_ERROR("setRefSpeeds failed.\n");
         return false;
    }

    //-- Enter position mode and perform movement
    std::vector<int> posModes(numRobotJoints, VOCAB_CM_POSITION);
    if (!iControlMode->setControlModes(posModes.data()))
    {
        CD_ERROR("setControlModes failed.\n");
        return false;
    }
    if ( ! iPositionControl->positionMove( qd.data() ) )
    {
        CD_ERROR("positionMove failed.\n");
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    cmcSuccess = true;
    CD_SUCCESS("Waiting\n");

    setCurrentState( VOCAB_CC_MOVJ_CONTROLLING );

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::relj(const std::vector<double> &xd)
{
    if ( referenceFrame == ICartesianSolver::TCP_FRAME )
    {
        return movj(xd);
    }

    int state;
    std::vector<double> x;
    if ( ! stat(state, x) )
    {
        CD_ERROR("stat failed.\n");
        return false;
    }
    for (unsigned int i = 0; i < xd.size(); i++)
    {
        x[i] += xd[i];
    }
    return movj(x);
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::movl(const std::vector<double> &xd)
{
    CD_WARNING("MOVL mode still experimental.\n");

    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    std::vector<double> x_base_tcp;
    if ( ! iCartesianSolver->fwdKin(currentQ,x_base_tcp) )
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }

    std::vector<double> xd_obj;
    if( referenceFrame == ICartesianSolver::TCP_FRAME )
    {
        if( ! iCartesianSolver->changeOrigin(xd, x_base_tcp, xd_obj) )
        {
            CD_ERROR("changeOrigin failed.\n");
            return false;
        }
    }
    else
    {
        xd_obj = xd;
    }

    //-- Create line trajectory
    iCartesianTrajectory = new KdlTrajectory;
    if( ! iCartesianTrajectory->setDuration(duration) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartesianTrajectory->addWaypoint(x_base_tcp) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartesianTrajectory->addWaypoint(xd_obj) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartesianTrajectory->configurePath( ICartesianTrajectory::LINE ) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartesianTrajectory->configureVelocityProfile( ICartesianTrajectory::TRAPEZOIDAL ) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartesianTrajectory->create() )
    {
        CD_ERROR("\n");
        return false;
    }

    //-- Set velocity mode and set state which makes rate thread implement control.
    std::vector<int> velModes(numRobotJoints, VOCAB_CM_VELOCITY);
    if (!iControlMode->setControlModes(velModes.data()))
    {
        CD_ERROR("setControlModes failed.\n");
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    movementStartTime = yarp::os::Time::now();
    cmcSuccess = true;
    CD_SUCCESS("Waiting\n");

    setCurrentState( VOCAB_CC_MOVL_CONTROLLING );

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::movv(const std::vector<double> &xdotd)
{
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    std::vector<double> x_base_tcp;
    if ( ! iCartesianSolver->fwdKin(currentQ,x_base_tcp) )
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }

    ICartesianTrajectory * iCartTraj = new KdlTrajectory;

    if( ! iCartTraj->addWaypoint(x_base_tcp, xdotd) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartTraj->configurePath( ICartesianTrajectory::LINE ) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartTraj->configureVelocityProfile( ICartesianTrajectory::RECTANGULAR ) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartTraj->create() )
    {
        CD_ERROR("\n");
        return false;
    }

    trajectoryMutex.wait();

    if( iCartesianTrajectory != NULL )
    {
        iCartesianTrajectory->destroy();
        delete iCartesianTrajectory;
    }

    //-- Transfer ownership.
    iCartesianTrajectory = iCartTraj;

    trajectoryMutex.post();

    //-- Set velocity mode and set state which makes rate thread implement control.
    std::vector<int> velModes(numRobotJoints, VOCAB_CM_VELOCITY);
    if (!iControlMode->setControlModes(velModes.data()))
    {
        CD_ERROR("setControlModes failed.\n");
        return false;
    }

    //-- Set state, enable CMC thread and wait for movement to be done
    movementStartTime = yarp::os::Time::now();
    cmcSuccess = true;
    CD_SUCCESS("Waiting\n");

    setCurrentState( VOCAB_CC_MOVV_CONTROLLING );

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::gcmp()
{
    //-- Set torque mode and set state which makes rate thread implement control.
    std::vector<int> torqModes(numRobotJoints, VOCAB_CM_TORQUE);
    if (!iControlMode->setControlModes(torqModes.data()))
    {
        CD_ERROR("setControlModes failed.\n");
        return false;
    }
    setCurrentState( VOCAB_CC_GCMP_CONTROLLING );
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::forc(const std::vector<double> &td)
{
    CD_WARNING("FORC mode still experimental.\n");

    if (referenceFrame == ICartesianSolver::TCP_FRAME)
    {
        CD_WARNING("TCP frame not supported yet in forc command.\n");
        return false;
    }

    //-- Set torque mode and set state which makes rate thread implement control.
    this->td = td;
    std::vector<int> torqModes(numRobotJoints, VOCAB_CM_TORQUE);
    if (!iControlMode->setControlModes(torqModes.data()))
    {
        CD_ERROR("setControlModes failed.\n");
        return false;
    }
    setCurrentState( VOCAB_CC_FORC_CONTROLLING );
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::stopControl()
{
    std::vector<int> posModes(numRobotJoints, VOCAB_CM_POSITION);
    if (!iControlMode->setControlModes(posModes.data()))
    {
        CD_ERROR("setControlModes failed.\n");
        return false;
    }
    iPositionControl->stop();
    setCurrentState( VOCAB_CC_NOT_CONTROLLING );
    if (iCartesianTrajectory != 0)
    {
        iCartesianTrajectory->destroy();
        delete iCartesianTrajectory;
        iCartesianTrajectory = 0;
    }
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::wait(double timeout)
{
    int state = getCurrentState();

    if (state != VOCAB_CC_MOVJ_CONTROLLING && state != VOCAB_CC_MOVL_CONTROLLING)
    {
        return true;
    }

    double start = yarp::os::Time::now();

    while (state != VOCAB_CC_NOT_CONTROLLING)
    {
        if (timeout != 0.0 && yarp::os::Time::now() - start > timeout)
        {
            CD_WARNING("Timeout reached (%f seconds), stopping control.\n", timeout);
            stopControl();
            break;
        }

        yarp::os::Time::delay(waitPeriodMs / 1000.0);
        state = getCurrentState();
    }

    return cmcSuccess;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::tool(const std::vector<double> &x)
{
    if ( ! iCartesianSolver->restoreOriginalChain() )
    {
        CD_ERROR("restoreOriginalChain failed\n");
        return false;
    }

    if ( ! iCartesianSolver->appendLink(x) )
    {
        CD_ERROR("appendLink failed\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::twist(const std::vector<double> &xdot)
{
    std::vector<int> velModes(numRobotJoints, VOCAB_CM_VELOCITY);
    if (!iControlMode->setControlModes(velModes.data()))
    {
        CD_ERROR("setControlModes failed.\n");
        return;
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    if ( ! iCartesianSolver->diffInvKin(currentQ, xdot, qdot, referenceFrame) )
    {
        CD_ERROR("diffInvKin failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > maxJointVelocity )
        {
            CD_ERROR("Maximum angular velocity hit: qdot[%d] = %f > %f [deg/s].\n", i, qdot[i], maxJointVelocity);
            std::fill(qdot.begin(), qdot.end(), 0.0);
            iVelocityControl->velocityMove(qdot.data());
            return;
        }
    }

    if ( ! iVelocityControl->velocityMove( qdot.data() ) )
    {
        CD_ERROR("velocityMove failed.\n");
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::pose(const std::vector<double> &x, double interval)
{
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    std::vector<double> x_base_tcp;
    if ( ! iCartesianSolver->fwdKin(currentQ,x_base_tcp) )
    {
        CD_ERROR("fwdKin failed.\n");
        return;
    }

    std::vector<double> xd_obj;
    if( referenceFrame == ICartesianSolver::TCP_FRAME )
    {
        if( ! iCartesianSolver->changeOrigin(x, x_base_tcp, xd_obj) )
        {
            CD_ERROR("changeOrigin failed.\n");
            return;
        }
    }
    else
    {
        xd_obj = x;
    }

    std::vector<double> xd;
    if ( ! iCartesianSolver->poseDiff(xd_obj, x_base_tcp, xd) )
    {
        CD_ERROR("fwdKinError failed.\n");
        return;
    }

    std::vector<double> xdot(xd.size());
    const double factor = gain / interval;
    std::transform(xd.begin(), xd.end(), xdot.begin(), std::bind1st(std::multiplies<double>(), factor));

    std::vector<int> velModes(numRobotJoints, VOCAB_CM_VELOCITY);
    if (!iControlMode->setControlModes(velModes.data()))
    {
        CD_ERROR("setControlModes failed.\n");
        return;
    }

    std::vector<double> qdot;
    if ( ! iCartesianSolver->diffInvKin(currentQ, xdot, qdot, referenceFrame) )
    {
        CD_ERROR("diffInvKin failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > maxJointVelocity )
        {
            CD_ERROR("Maximum angular velocity hit: qdot[%d] = %f > %f [deg/s].\n", i, qdot[i], maxJointVelocity);
            std::fill(qdot.begin(), qdot.end(), 0.0);
            iVelocityControl->velocityMove(qdot.data());
            return;
        }
    }

    if ( ! iVelocityControl->velocityMove( qdot.data() ) )
    {
        CD_ERROR("velocityMove failed.\n");
        return;
    }
}

// -----------------------------------------------------------------------------

void roboticslab::BasicCartesianControl::movi(const std::vector<double> &x)
{
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    std::vector<double> q;
    if ( ! iCartesianSolver->invKin(x,currentQ,q,referenceFrame) )
    {
        CD_ERROR("invKin failed.\n");
        return;
    }

    std::vector<int> posdModes(numRobotJoints, VOCAB_CM_POSITION_DIRECT);
    if ( ! iControlMode->setControlModes(posdModes.data()) )
    {
        CD_ERROR("setControlModes failed.\n");
        return;
    }

    if ( ! iPositionDirect->setPositions( q.data() ) )
    {
        CD_ERROR("setPositions failed.\n");
        return;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::setParameter(int vocab, double value)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        CD_ERROR("Unable to set config parameter while controlling.\n");
        return false;
    }

    switch (vocab)
    {
    case VOCAB_CC_CONFIG_GAIN:
        if (value < 0.0)
        {
            CD_ERROR("Controller gain cannot be negative.\n");
            return false;
        }
        gain = value;
        break;
    case VOCAB_CC_CONFIG_MAX_JOINT_VEL:
        if (value <= 0.0)
        {
            CD_ERROR("Maximum joint velocity cannot be negative nor zero.\n");
            return false;
        }
        maxJointVelocity = value;
        break;
    case VOCAB_CC_CONFIG_TRAJ_DURATION:
        if (value <= 0.0)
        {
            CD_ERROR("Trajectory duration cannot be negative nor zero.\n");
            return false;
        }
        duration = value;
        break;
    case VOCAB_CC_CONFIG_CMC_RATE:
        if (!yarp::os::RateThread::setRate(value))
        {
            CD_ERROR("Cannot set new CMC rate.\n");
            return false;
        }
        cmcRateMs = value;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        if (value <= 0.0)
        {
            CD_ERROR("Wait period cannot be negative nor zero.\n");
            return false;
        }
        waitPeriodMs = value;
        break;
    case VOCAB_CC_CONFIG_FRAME:
        if (value != ICartesianSolver::BASE_FRAME && value != ICartesianSolver::TCP_FRAME)
        {
            CD_ERROR("Unrecognized of unsupported reference frame vocab.\n");
            return false;
        }
        referenceFrame = static_cast<ICartesianSolver::reference_frame>(value);
        break;
    default:
        CD_ERROR("Unrecognized or unsupported config parameter key: %s.\n", yarp::os::Vocab::decode(vocab).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::getParameter(int vocab, double * value)
{
    switch (vocab)
    {
    case VOCAB_CC_CONFIG_GAIN:
        *value = gain;
        break;
    case VOCAB_CC_CONFIG_MAX_JOINT_VEL:
        *value = maxJointVelocity;
        break;
    case VOCAB_CC_CONFIG_TRAJ_DURATION:
        *value = duration;
        break;
    case VOCAB_CC_CONFIG_CMC_RATE:
        *value = cmcRateMs;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        *value = waitPeriodMs;
        break;
    case VOCAB_CC_CONFIG_FRAME:
        *value = referenceFrame;
        break;
    default:
        CD_ERROR("Unrecognized or unsupported config parameter key: %s.\n", yarp::os::Vocab::decode(vocab).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::setParameters(const std::map<int, double> & params)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        CD_ERROR("Unable to set config parameters while controlling.\n");
        return false;
    }

    bool ok = true;

    for (std::map<int, double>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
        ok &= setParameter(it->first, it->second);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::getParameters(std::map<int, double> & params)
{
    params.insert(std::pair<int, double>(VOCAB_CC_CONFIG_GAIN, gain));
    params.insert(std::pair<int, double>(VOCAB_CC_CONFIG_MAX_JOINT_VEL, maxJointVelocity));
    params.insert(std::pair<int, double>(VOCAB_CC_CONFIG_TRAJ_DURATION, duration));
    params.insert(std::pair<int, double>(VOCAB_CC_CONFIG_CMC_RATE, cmcRateMs));
    params.insert(std::pair<int, double>(VOCAB_CC_CONFIG_WAIT_PERIOD, waitPeriodMs));
    params.insert(std::pair<int, double>(VOCAB_CC_CONFIG_FRAME, referenceFrame));
    return true;
}

// -----------------------------------------------------------------------------

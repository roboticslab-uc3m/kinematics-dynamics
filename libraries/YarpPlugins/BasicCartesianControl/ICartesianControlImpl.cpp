// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath>  //-- std::abs
#include <algorithm>
#include <functional>

#include <ColorDebug.hpp>

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
    if ( ! iCartesianSolver->invKin(xd,currentQ,q) )
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
    if ( ! iCartesianSolver->invKin(xd,currentQ,qd) )
    {
        CD_ERROR("invKin failed.\n");
        return false;
    }

    //-- Find out the maximum time to move
    double max_time = 0;
    for(unsigned int joint=0;joint<numSolverJoints;joint++)
    {
        CD_INFO("dist[%d]: %f\n",joint,std::abs(qd[joint]-currentQ[joint]));
        if (std::abs((qd[joint]-currentQ[joint]) / MAX_ANG_VEL) > max_time)
        {
            max_time = std::abs( (qd[joint]-currentQ[joint]) / MAX_ANG_VEL);
            CD_INFO(" -->candidate: %f\n",max_time);
        }
    }
    CD_INFO("max_time[final]: %f\n",max_time);

    //-- Compute, store old and set joint velocities given this time
    std::vector<double> vmo, vmoStored(numRobotJoints);
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
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        if ( ! iControlMode->setPositionMode(joint) )
        {
            CD_ERROR("setPositionMode failed at joint %d.\n", joint);
            return false;
        }
    }
    if ( ! iPositionControl->positionMove( qd.data() ) )
    {
        CD_ERROR("positionMove failed.\n");
        return false;
    }

    //-- Set state, perform and wait for movement to be done
    setCurrentState( VOCAB_CC_MOVJ_CONTROLLING );

    CD_SUCCESS("Waiting\n");
    bool done = false;
    while(!done)
    {
        iPositionControl->checkMotionDone(&done);
        printf(".");
        fflush(stdout);
        yarp::os::Time::delay(0.5);
    }

    //-- Reestablish state and velocities
    setCurrentState( VOCAB_CC_NOT_CONTROLLING );

    if ( ! iPositionControl->setRefSpeeds( vmoStored.data() ) )
    {
         CD_ERROR("setRefSpeeds (to restore) failed.\n");
         return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::relj(const std::vector<double> &xd)
{
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

    int state;
    std::vector<double> x;
    if ( ! stat(state, x) )
    {
        CD_ERROR("stat failed.\n");
        return false;
    }

    //-- Create line trajectory
    iCartesianTrajectory = new KdlTrajectory;
    if( ! iCartesianTrajectory->setDuration(duration) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartesianTrajectory->addWaypoint(x) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartesianTrajectory->addWaypoint(xd) )
    {
        CD_ERROR("\n");
        return false;
    }
    if( ! iCartesianTrajectory->configurePath( ICartesianTrajectory::LINE ) )
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
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }
    movementStartTime = yarp::os::Time::now();
    setCurrentState( VOCAB_CC_MOVL_CONTROLLING );

    //-- Wait for movement to be done, then delete
    CD_SUCCESS("Waiting\n");
    while( getCurrentState() == VOCAB_CC_MOVL_CONTROLLING )
    {
        printf(".");
        fflush(stdout);
        yarp::os::Time::delay(0.5);
    }
    iCartesianTrajectory->destroy();
    delete iCartesianTrajectory;
    iCartesianTrajectory = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::movv(const std::vector<double> &xdotd)
{
    CD_WARNING("MOVV mode still experimental.\n");

    //-- Set velocity mode and set state which makes rate thread implement control.
    this->xdotd = xdotd;
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }
    setCurrentState( VOCAB_CC_MOVV_CONTROLLING );
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::gcmp()
{
    //-- Set torque mode and set state which makes rate thread implement control.
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setTorqueMode(joint);
    }
    setCurrentState( VOCAB_CC_GCMP_CONTROLLING );
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::forc(const std::vector<double> &td)
{
    CD_WARNING("FORC mode still experimental.\n");

    //-- Set torque mode and set state which makes rate thread implement control.
    this->td = td;
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setTorqueMode(joint);
    }
    setCurrentState( VOCAB_CC_FORC_CONTROLLING );
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::stopControl()
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setPositionMode(joint);
    }
    iPositionControl->stop();
    setCurrentState( VOCAB_CC_NOT_CONTROLLING );
    return true;
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

void roboticslab::BasicCartesianControl::fwd(const std::vector<double> &rot, double step)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    std::vector<double> xdotee(6);
    xdotee[2] = std::max(step, 0.0);
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if ( ! iCartesianSolver->diffInvKinEE( currentQ, xdotee, qdot ) )
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
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

void roboticslab::BasicCartesianControl::bkwd(const std::vector<double> &rot, double step)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    std::vector<double> xdotee(6);
    xdotee[2] = -std::max(step, 0.0);
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if ( ! iCartesianSolver->diffInvKinEE( currentQ, xdotee, qdot ) )
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
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

void roboticslab::BasicCartesianControl::rot(const std::vector<double> &rot)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    std::vector<double> xdotee(6);
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if ( ! iCartesianSolver->diffInvKinEE( currentQ, xdotee, qdot ) )
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
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

void roboticslab::BasicCartesianControl::pan(const std::vector<double> &transl)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    std::vector<double> xdotee(6);
    xdotee[0] = transl[0];
    xdotee[1] = transl[1];
    xdotee[2] = transl[2];

    if ( ! iCartesianSolver->diffInvKinEE( currentQ, xdotee, qdot ) )
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
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

void roboticslab::BasicCartesianControl::vmos(const std::vector<double> &xdot)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    if ( ! iCartesianSolver->diffInvKin( currentQ, xdot, qdot ) )
    {
        CD_ERROR("diffInvKin failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
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

void roboticslab::BasicCartesianControl::eff(const std::vector<double> &xdotee)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    if ( ! iCartesianSolver->diffInvKinEE( currentQ, xdotee, qdot ) )
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
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

    std::vector<double> xd;
    if ( ! iCartesianSolver->fwdKinError(x, currentQ, xd) )
    {
        CD_ERROR("fwdKinError failed.\n");
        return;
    }

    std::vector<double> xdot(xd.size());
    const double factor = DEFAULT_GAIN / interval;
    std::transform(xd.begin(), xd.end(), xdot.begin(), std::bind1st(std::multiplies<double>(), factor));

    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> qdot;
    if ( ! iCartesianSolver->diffInvKin(currentQ, xdot, qdot) )
    {
        CD_ERROR("diffInvKin failed.\n");
        return;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
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

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

#include <cmath>  //-- std::abs
#include <algorithm>
#include <functional>

#include <ColorDebug.hpp>

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
    trajectory.newLine(x,xd);

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
    trajectory.deleteLine();

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

bool roboticslab::BasicCartesianControl::fwd(const std::vector<double> &rot, double step)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    std::vector<double> xdotee(6);
    xdotee[2] = step;
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if ( ! iCartesianSolver->diffInvKinEE( currentQ, xdotee, qdot ) )
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return false;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
            return false;
        }
    }

    if ( ! iVelocityControl->velocityMove( qdot.data() ) )
    {
        CD_ERROR("velocityMove failed.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::bkwd(const std::vector<double> &rot, double step)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    std::vector<double> xdotee(6);
    xdotee[2] = -step;
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if ( ! iCartesianSolver->diffInvKinEE( currentQ, xdotee, qdot ) )
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return false;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
            return false;
        }
    }

    if ( ! iVelocityControl->velocityMove( qdot.data() ) )
    {
        CD_ERROR("velocityMove failed.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::rot(const std::vector<double> &rot)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    std::vector<double> xdotee(6);
    xdotee[3] = rot[0];
    xdotee[4] = rot[1];
    xdotee[5] = rot[2];

    if ( ! iCartesianSolver->diffInvKinEE( currentQ, xdotee, qdot ) )
    {
        CD_ERROR("diffInvKinEE failed.\n");
        return false;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
            return false;
        }
    }

    if ( ! iVelocityControl->velocityMove( qdot.data() ) )
    {
        CD_ERROR("velocityMove failed.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::vmos(const std::vector<double> &xdot)
{
    for (unsigned int joint = 0; joint < numRobotJoints; joint++)
    {
        iControlMode->setVelocityMode(joint);
    }

    std::vector<double> currentQ(numRobotJoints), qdot;
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    if ( ! iCartesianSolver->diffInvKin( currentQ, xdot, qdot ) )
    {
        CD_ERROR("diffInvKin failed.\n");
        return false;
    }

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        if ( std::abs(qdot[i]) > MAX_ANG_VEL )
        {
            CD_ERROR("Maximum angular velocity hit at joint %d (qdot[%d] = %f > %f [deg/s]).\n", i + 1, i, qdot[i], MAX_ANG_VEL);
            return false;
        }
    }

    if ( ! iVelocityControl->velocityMove( qdot.data() ) )
    {
        CD_ERROR("velocityMove failed.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::BasicCartesianControl::pose(const std::vector<double> &x, double interval)
{
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    std::vector<double> xd;
    if ( ! iCartesianSolver->fwdKinError(x, currentQ, xd) )
    {
        CD_ERROR("fwdKinError failed.\n");
        return false;
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
        return false;
    }

    if ( ! iVelocityControl->velocityMove( qdot.data() ) )
    {
        CD_ERROR("velocityMove failed.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

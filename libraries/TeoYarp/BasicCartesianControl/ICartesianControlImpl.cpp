// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool teo::BasicCartesianControl::stat(int &state, std::vector<double> &x)
{
    std::vector<double> qCurrent(numRobotJoints);
    if ( ! iEncoders->getEncoders( qCurrent.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->fwdKin(qCurrent,x) )
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }
    state = currentState;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    std::vector<double> qCurrent(numRobotJoints);
    if ( ! iEncoders->getEncoders( qCurrent.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->invKin(xd,qCurrent,q) )
    {
        CD_ERROR("invKin failed.\n");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::movj(const std::vector<double> &xd)
{
    std::vector<double> qCurrent(numRobotJoints), q;
    if ( ! iEncoders->getEncoders( qCurrent.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->invKin(xd,qCurrent,q) )
    {
        CD_ERROR("invKin failed.\n");
        return false;
    }

    //-- Find out the maximum time to move
    double max_time = 0;
    for(unsigned int joint=0;joint<numRobotJoints;joint++)
    {
        CD_INFO("dist[%d]: %f\n",joint,fabs(q[joint]-qCurrent[joint]));
        if (fabs((q[joint]-qCurrent[joint]) / MAX_ANG_VEL) > max_time)
        {
            max_time = fabs( (q[joint]-qCurrent[joint]) / MAX_ANG_VEL);
            CD_INFO(" -->candidate: %f\n",max_time);
        }
    }
    CD_INFO("max_time[final]: %f\n",max_time);

    //-- Compute, store old and set joint velocities given this time
    std::vector<double> vmo, vmoStored;
    for(unsigned int joint=0;joint<numRobotJoints;joint++)
    {
        vmo.push_back( fabs(q[joint] - qCurrent[joint])/max_time );
        CD_INFO("vmo[%d]: %f\n",joint,vmo[joint]);
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
    if ( ! iPositionControl->setPositionMode() )
    {
        CD_ERROR("setPositionMode failed.\n");
        return false;
    }
    if ( ! iPositionControl->positionMove( q.data() ) )
    {
        CD_ERROR("positionMove failed.\n");
        return false;
    }

    //-- Set state, perform and wait for movement to be done
    currentState = VOCAB_CC_MOVJ_CONTROLLING;

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
    currentState = VOCAB_CC_NOT_CONTROLLING;

    if ( ! iPositionControl->setRefSpeeds( vmoStored.data() ) )
    {
         CD_ERROR("setRefSpeeds (to restore) failed.\n");
         return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::movl(const std::vector<double> &xd)
{
    std::vector<double> qCurrent(numRobotJoints), q;
    if ( ! iEncoders->getEncoders( qCurrent.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }
    if ( ! iCartesianSolver->invKin(xd,qCurrent,q) )
    {
        CD_ERROR("invKin failed.\n");
        return false;
    }

    movementStartTime = yarp::os::Time::now();
    currentState = VOCAB_CC_MOVL_CONTROLLING;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::stop()
{
    iPositionControl->stop();
    currentState = VOCAB_CC_NOT_CONTROLLING;
    return true;
}

// -----------------------------------------------------------------------------

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

    // Find out the maximum time to move
    double max_time = 0;
    for(unsigned int motor=0;motor<numRobotJoints;motor++) {
        CD_INFO("dist[%d]: %f\n",motor,fabs(q[motor]-qCurrent[motor]));
        if (fabs((q[motor]-qCurrent[motor]) / MAX_ANG_VEL) > max_time)
        {
            max_time = fabs( (q[motor]-qCurrent[motor]) / MAX_ANG_VEL);
            CD_INFO(" -->candidate: %f\n",max_time);
        }
    }
    CD_INFO("max_time[final]: %f\n",max_time);

    std::vector<double> vmo;
    for(unsigned int motor=0;motor<numRobotJoints;motor++) {
        vmo.push_back( fabs(q[motor] - qCurrent[motor])/max_time );
        CD_INFO("vmo[%d]: %f\n",motor,vmo[motor]);
    }
    if ( ! iPositionControl->setRefSpeeds( vmo.data() ) )
    {
         CD_ERROR("setRefSpeeds failed.\n");
         return false;
    }
    if ( ! iPositionControl->positionMove( q.data() ) )
    {
        CD_ERROR("positionMove failed.\n");
        return false;
    }

    currentState = VOCAB_CC_MOVJ_CONTROLLING;

    CD_SUCCESS("Waiting\n");
    bool done = false;
    while(!done) {
        iPositionControl->checkMotionDone(&done);
        printf(".");
        fflush(stdout);
        yarp::os::Time::delay(0.5);
    }

    currentState = VOCAB_CC_NOT_CONTROLLING;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::movl(const std::vector<double> &xd)
{
    currentState = VOCAB_CC_MOVL_CONTROLLING;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BasicCartesianControl::stop()
{
    currentState = VOCAB_CC_NOT_CONTROLLING;
    return true;
}

// -----------------------------------------------------------------------------

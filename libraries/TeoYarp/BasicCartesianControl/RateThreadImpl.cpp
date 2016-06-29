// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- RateThread Related ------------------------------------

void teo::BasicCartesianControl::run() {

    if (currentState == VOCAB_CC_MOVL_CONTROLLING)
    {
        double movementTime = yarp::os::Time::now() - movementStartTime;
        CD_DEBUG("MOVEL_CONTROLLING: %f\n",movementTime);

        std::vector<double> currentQ(numRobotJoints), currentX, commandXdot, commandQdot;

        if ( ! iEncoders->getEncoders( currentQ.data() ) )
        {
            CD_WARNING("getEncoders failed, not updating control this iteration.\n");
            return;
        }
        if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )
        {
            CD_WARNING("fwdKin failed, not updating control this iteration.\n");
            return;
        }

        //KDL::Frame desiredF = movementTrajectory->Pos(movementTime);
        //KDL::Twist desiredT = movementTrajectory->Vel(movementTime);

        //KDL::Twist commandXdot = diff(currentX, targetF);
        //for(unsigned int i=0; i<6; i++)
        //{
        //    commandXdot(i) *= GAIN;
        //    commandXdot(i) += desiredT(i);
        //}

        if (! iCartesianSolver->diffInvKin(currentQ,commandXdot,commandQdot) )
        {
            CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        }
        if( ! iVelocityControl->velocityMove( commandQdot.data() ) )
        {
            CD_WARNING("velocityMove failed, not updating control this iteration.\n");
        }

    }
    return;
}

// -----------------------------------------------------------------------------


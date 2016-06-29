// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- RateThread Related ------------------------------------

void teo::BasicCartesianControl::run() {

    if (currentState == VOCAB_CC_MOVL_CONTROLLING)
    {
        double movementTime = yarp::os::Time::now() - movementStartTime;
        CD_DEBUG("MOVEL_CONTROLLING: %f\n",movementTime);

        //-- Obtain current joint position, use to compute current Cartesian position.
        std::vector<double> currentQ(numRobotJoints), currentX;
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

        //-- Obtain desired Cartesian position and velocity.
        std::vector<double> desiredX, desiredXdot;
        trajectory.getX(movementTime, desiredX);
        trajectory.getXdot(movementTime, desiredXdot);

        //-- Apply control law to compute robot joint velocity commands.
        std::vector<double> commandXdot, commandQdot;
        //KDL::Twist commandXdot = diff(currentX, desiredX);
        //for(unsigned int i=0; i<6; i++)
        //{
        //    commandXdot(i) *= GAIN;
        //    commandXdot(i) += desiredXdot(i);
        //}
        if (! iCartesianSolver->diffInvKin(currentQ,commandXdot,commandQdot) )
        {
            CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        }

        //-- Send robot in joint velocity commands.
        if( ! iVelocityControl->velocityMove( commandQdot.data() ) )
        {
            CD_WARNING("velocityMove failed, not updating control this iteration.\n");
        }

    }
    return;
}

// -----------------------------------------------------------------------------


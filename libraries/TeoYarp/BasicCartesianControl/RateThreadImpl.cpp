// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BasicCartesianControl.hpp"

// ------------------- RateThread Related ------------------------------------

void teo::BasicCartesianControl::run() {

    if (currentState == VOCAB_CC_MOVL_CONTROLLING)
    {
        double movementTime = yarp::os::Time::now() - movementStartTime;

        if( movementTime > DEFAULT_DURATION )
        {
            this->stopControl();
            return;
        }

        CD_DEBUG("MOVEL_CONTROLLING: %f\n",movementTime);

        //-- Obtain current joint position
        std::vector<double> currentQ(numRobotJoints);
        if ( ! iEncoders->getEncoders( currentQ.data() ) )
        {
            CD_WARNING("getEncoders failed, not updating control this iteration.\n");
            return;
        }

        //-- Obtain desired Cartesian position and velocity.
        std::vector<double> desiredX, desiredXdot;
        trajectory.getX(movementTime, desiredX);
        trajectory.getXdot(movementTime, desiredXdot);

        //-- Apply control law to compute robot Cartesian velocity commands.
        std::vector<double> commandXdot;
        iCartesianSolver->fwdKinError(desiredX,currentQ, commandXdot);
        for(unsigned int i=0; i<6; i++)
        {
            commandXdot[i] *= -DEFAULT_GAIN;
            commandXdot[i] += desiredXdot[i];
        }

        //-- Compute joint velocity commands and send to robot.
        std::vector<double> commandQdot;
        if (! iCartesianSolver->diffInvKin(currentQ,commandXdot,commandQdot) )
        {
            CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        }
        if( ! iVelocityControl->velocityMove( commandQdot.data() ) )
        {
            CD_WARNING("velocityMove failed, not updating control this iteration.\n");
        }

    }
    else if (currentState == VOCAB_CC_MOVV_CONTROLLING)
    {
        //-- Obtain current joint position
        std::vector<double> currentQ(numRobotJoints);
        if ( ! iEncoders->getEncoders( currentQ.data() ) )
        {
            CD_WARNING("getEncoders failed, not updating control this iteration.\n");
            return;
        }

        //-- Compute joint velocity commands and send to robot.
        std::vector<double> commandQdot;
        if (! iCartesianSolver->diffInvKin(currentQ,xdotd,commandQdot) )
        {
            CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        }
        if( ! iVelocityControl->velocityMove( commandQdot.data() ) )
        {
            CD_WARNING("velocityMove failed, not updating control this iteration.\n");
        }
    }
    else if (currentState == VOCAB_CC_GCMP_CONTROLLING)
    {
        //-- Obtain current joint position
        std::vector<double> currentQ(numRobotJoints);
        if ( ! iEncoders->getEncoders( currentQ.data() ) )
        {
            CD_WARNING("getEncoders failed, not updating control this iteration.\n");
            return;
        }

        std::vector< double > t(numRobotJoints);
        iCartesianSolver->invDyn(currentQ,t);

        iTorqueControl->setRefTorques( t.data() );
    }
    else if (currentState == VOCAB_CC_FORC_CONTROLLING)
    {
        //-- Obtain current joint position
        std::vector<double> currentQ(numRobotJoints);
        if ( ! iEncoders->getEncoders( currentQ.data() ) )
        {
            CD_WARNING("getEncoders failed, not updating control this iteration.\n");
            return;
        }

        std::vector<double> qdot(numRobotJoints,0), qdotdot(numRobotJoints,0);
        std::vector< std::vector<double> > fexts;
        for (int i=0; i<numRobotJoints-1; i++)  //-- "numRobotJoints-1" is important
        {
            std::vector<double> fext(6,0);
            fexts.push_back(fext);
        }
        fexts.push_back(td);

        std::vector< double > t(numRobotJoints);
        iCartesianSolver->invDyn(currentQ,qdot,qdotdot,fexts,t);

        iTorqueControl->setRefTorques( t.data() );
    }

    return;
}

// -----------------------------------------------------------------------------


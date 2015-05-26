
// -----------------------------------------------------------------------------

#include "KdlController.hpp"

// ------------------- RateThread Related ------------------------------------

bool teo::KdlController::threadInit() {
    CD_SUCCESS("Begin. Started %f ms ratethread\n",getRate());
    double dRealUnits[cmcNumMotors];
    while(!enc->getEncoders(dRealUnits)) {
        CD_INFO("Wait for getEncoders()...\n");
        yarp::os::Time::delay(0.5);
    }
    CD_SUCCESS("Ok getEncoders().\n");
    yarp::sig::Vector realUnits(cmcNumMotors,dRealUnits);
    yarp::sig::Vector x,o;
    fwdKin(realUnits,x,o);
    CD_INFO("End.\n");
    return true;
}

// -----------------------------------------------------------------------------

void teo::KdlController::run() {
    if (cmc_status>0) {  // If it is movement
        double dRealUnits[100];
        if( ! enc->getEncoders(dRealUnits) ) {
            CD_WARNING("getEncoders failed.\n");
            return;  // bad practice??
        }
        yarp::sig::Vector realUnits(cmcNumMotors,dRealUnits);
        yarp::sig::Vector x,o;
        fwdKin(realUnits,x,o);

        KDL::Frame currentF;
        currentF.p.data[0]=x[0];
        currentF.p.data[1]=x[1];
        currentF.p.data[2]=x[2];
        if (angleRepr == "eulerYZ") {  // ASIBOT
            currentF.M = Rotation::EulerZYZ(::atan2(x[1],x[0]),toRad(o[0]), toRad(o[1]));
        } else if (angleRepr == "eulerZYZ") {
            currentF.M = Rotation::EulerZYZ(toRad(o[0]), toRad(o[1]), toRad(o[2]));
        } else if (angleRepr == "RPY") {
            currentF.M = Rotation::RPY(toRad(o[0]), toRad(o[1]), toRad(o[2]));
        } else {  // axisAngle, etc.
            CD_WARNING("KDL no compatible angleRepr with: %s\n",angleRepr.c_str());
        }

        bool done = true;
        /*for(int s=0;s<3;s++) {
            printf("fabs(x[%d]-targetF.p.data[%d]: %f\n",s,s,fabs(x[s]-targetF.p.data[s]));
            if(fabs(x[s]-targetF.p.data[s])>CARTPOS_PRECISION) done = false;
        }
        for(int s=0;s<o.size();s++) {
            printf("fabs(o[%d]-targetO[%d]: %f\n",s,s,fabs(o[s]-targetO[s]));
            if(fabs(o[s]-targetO[s])>CARTORI_PRECISION) done = false;
        }*/
        if(!Equal(currentF,targetF,epsilon)) done = false;
        
        if (done) {
            printf("Target reached in %f.\n",Time::now()-startTime);
            startTime = 0;
            pos->setPositionMode();
            cmc_status=0;
//            delete _orient;
//            _orient=0;
        } else {
            //printf("Inside control loop moving.\n");
 
/*            yarp::sig::Vector xP,xPd,xPdotd,eP,lawxP;
            lawxP.resize(3);
            lawxP = (eP * GAIN * (cmcMs/1000.0)) + xPdotd;  // GAIN=0 => lawxP = xPdotd;
            t = Ja_pinv * lawxP;*/

//            double _eps = 0.00001;
//            int _maxiter = 150;
//            ChainIkSolverVel_wdls iksolverv_wdls(theChain, _eps, _maxiter);
//            int ret_ik_v = iksolverv_wdls.CartToJnt(q_current,T_current,qdot_current);

            double sTime = Time::now()-startTime;
            if(sTime>currentTrajectory->Duration()){
                printf ("[warning] out of time at %f.\n",sTime);
                startTime = 0;
                pos->setPositionMode();
                cmc_status=0;
                return;  // bad practice??
            }

            //KDL::Frame desiredF = currentTrajectory->Pos(sTime);  // Unused in current control law.
            KDL::Twist desiredT = currentTrajectory->Vel(sTime);

            /*JntArray currentRads = JntArray(cmcNumMotors);
            for (int motor=0; motor<cmcNumMotors; motor++) {
                if(isPrismatic[motor]) currentRads(motor)=dRealUnits[motor];
                else currentRads(motor)=toRad(dRealUnits[motor]);
            }*/
            JntArray currentRads = JntArray(vectorOfCmcMotorIdxs.size() );
            for (int idxIdx=0; idxIdx<vectorOfCmcMotorIdxs.size(); idxIdx++) {
                int motor = vectorOfCmcMotorIdxs[idxIdx];
                if(isPrismatic[idxIdx]) currentRads(idxIdx)=dRealUnits[motor];
                else currentRads(idxIdx)=toDeg(dRealUnits[motor]);
                //CD_INFO("Set qdot[%d] = kdlqdot[%d] = %f.\n",motor,idxIdx, qdot[motor]);
            }

            KDL::Twist currentT = diff(currentF, targetF);
            for(unsigned int i=0; i<6; i++) {
                currentT(i) *= GAIN;
                currentT(i) += desiredT(i);
            }

            //JntArray kdlqdot = JntArray(cmcNumMotors);
            JntArray kdlqdot = JntArray( vectorOfCmcMotorIdxs.size() );

            ChainIkSolverVel_pinv iksolverv(theChain);
            int ret = iksolverv.CartToJnt(currentRads,currentT,kdlqdot);
            if (ret < 0) {
                CD_WARNING("iksolverv return < 0, returning.\n");
                return;
            }

            double qdot[cmcNumMotors];
            /*for (int motor=0; motor<cmcNumMotors; motor++) {
                if(isPrismatic[motor]) qdot[motor]=kdlqdot(motor);
                else qdot[motor]=toDeg(kdlqdot(motor));
            }*/
            for (int motor=0; motor<cmcNumMotors; motor++) {
                qdot[motor] = 0.0;
            }
            for (int idxIdx=0; idxIdx<vectorOfCmcMotorIdxs.size(); idxIdx++) {
                int motor = vectorOfCmcMotorIdxs[idxIdx];
                if(isPrismatic[motor]) qdot[motor]=kdlqdot(idxIdx);
                else qdot[motor]=toDeg(kdlqdot(idxIdx));
                //CD_INFO("Set qdot[%d] = kdlqdot[%d] = %f.\n",motor,idxIdx, qdot[motor]);
            }

            if( ! vel->velocityMove(qdot) ) {
                CD_WARNING("Could not send velocity!!!\n");
            }

        }
    } else {  // If it is stopped or breaked, reamain unchanged
        // printf("Inside control loop stopped.\n");
    }
}

// -----------------------------------------------------------------------------


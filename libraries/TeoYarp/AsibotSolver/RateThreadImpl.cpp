// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <AsibotSolver.hpp>

// ------------------- RateThread Related ------------------------------------

void roboticslab::AsibotSolver::run() {
    using namespace yarp::math;
    if (cmc_status>0) {  // If it is movement
        double realDeg[NUM_MOTORS];
        if(!enc->getEncoders(realDeg)) {
            fprintf(stderr, "[CartesianBot] warning: getPose() failed to getEncoders()\n");
            return;  // bad practice??
        }
        yarp::sig::Vector x,o;
        //fwdKin(realDeg,x,o);
        bool done = true;
        if(fabs(x[0]-targetX[0])>CARTPOS_PRECISION) done = false;
        if(fabs(x[1]-targetX[1])>CARTPOS_PRECISION) done = false;
        if(fabs(x[2]-targetX[2])>CARTPOS_PRECISION) done = false;
        if((withOri)&&(fabs(o[0]-targetO[0])>CARTORI_PRECISION)) done = false;
        if((withOri)&&(fabs(o[1]-targetO[1])>CARTORI_PRECISION)) done = false;
        if (done) {
            printf("[CartesianBot] success: Target reached in %f.\n",yarp::os::Time::now()-startTime);
            delete trajPrP;
            trajPrP = 0;
            delete trajPhP;
            trajPhP = 0;
            delete trajOz;
            trajOz = 0;
            delete trajOyP;
            trajOyP = 0;
            delete trajOzPP;
            trajOzPP = 0;
            startTime = 0;
            for (int i=0; i<NUM_MOTORS; i++)
                mode->setPositionMode(i);
            cmc_status=0;
        } else {
            //printf("Inside control loop moving.\n");
            yarp::sig::Vector xP,xPd,xPdotd,eP,lawxP;
            double ozRad = atan2(x[1],x[0]);
            xP.push_back(sqrt(x[0]*x[0]+x[1]*x[1]));  // prP
            xP.push_back(x[2]-A0);  // phP
            xP.push_back(toRad(o[0]));  // oyP
            //printf("Problem statement:\n");
            //printf("oz: %f\nxP: %f\nzP: %f\n",toDeg(ozRad),xP[0],xP[1]);
            double sTime = yarp::os::Time::now()-startTime;
            if(sTime>trajPrP->getT()){
                printf ("[CartesianBot] warning: Out of time at %f.\n",sTime);
                startTime = 0;
                for (int i=0; i<NUM_MOTORS; i++)
                    mode->setPositionMode(i);
                cmc_status=0;
                return;  // bad practice??
            }
            xPd.push_back(trajPrP->get(sTime));
            xPd.push_back(trajPhP->get(sTime));
            xPd.push_back(toRad(trajOyP->get(sTime)));
            eP.resize(3);
            eP = xPd - xP;
            xPdotd.push_back(trajPrP->getdot(sTime));
            xPdotd.push_back(trajPhP->getdot(sTime));
            xPdotd.push_back(toRad(trajOyP->getdot(sTime)));
            lawxP.resize(3);
            lawxP = (eP * GAIN * (cmcMs/1000.0)) + xPdotd;  // GAIN=0 => lawxP = xPdotd;
            yarp::sig::Matrix Ja(3,3);
            for (int i=0; i<NUM_MOTORS; i++)
                realRad[i]=toRad(realDeg[i]);
            Ja(0,0) = A2*cos(realRad[1]+realRad[2]) + A1*cos(realRad[1]) + A3*cos(realRad[1]+realRad[2]+realRad[3]);
            Ja(0,1) = A2*cos(realRad[1]+realRad[2]) + A3*cos(realRad[1]+realRad[2]+realRad[3]);
            Ja(0,2) = A3*cos(realRad[1]+realRad[2]+realRad[3]);
            Ja(1,0) = -A2*sin(realRad[1]+realRad[2]) - A1*sin(realRad[1]) - A3*sin(realRad[1]+realRad[2]+realRad[3]);
            Ja(1,1) = -A2*sin(realRad[1]+realRad[2]) - A3*sin(realRad[1]+realRad[2]+realRad[3]);
            Ja(1,2) = -A3*sin(realRad[1]+realRad[2]+realRad[3]);
            Ja(2,0) = 1;
            Ja(2,1) = 1;
            Ja(2,2) = 1;
            yarp::sig::Matrix Ja_pinv(yarp::math::pinv(Ja,1.0e-2));
//            yarp::sig::Matrix Ja_pinv(pinv(Ja));
            yarp::sig::Vector t;
            t.resize(3);
            t = Ja_pinv * lawxP;
            double qdot[NUM_MOTORS];
            double eoz = trajOz->get(sTime) - realDeg[0];
            qdot[0] = trajOz->getdot(sTime) + GAIN*(cmcMs/1000.0)*eoz;  // lawoz
            qdot[1] = toDeg(t[0]);
            qdot[2] = toDeg(t[1]);
            qdot[3] = toDeg(t[2]);
            double eOzPP = trajOzPP->get(sTime) - realDeg[4];
            qdot[4] = trajOzPP->getdot(sTime) + GAIN*(cmcMs/1000.0)*eOzPP;  // lawOzP
            if(!vel->velocityMove(qdot))
                fprintf(stderr, "[CartesianBot] warning: FAILED ON VELOCITY MOVE!!!\n");
        }
    } /*else {  // If it is stopped or breaked, remain unchanged
        printf("Inside control loop stopped.\n");
    }*/
}

// -----------------------------------------------------------------------------


// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <AsibotSolver.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::goToPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t) {
    using namespace yarp::math;  // just for matrix operators
    yarp::sig::Vector x,o;  // empty vectors
    //getPose(x,o);  // where we put the result of performing fwd kinematics of current position
    //if (!isQuiet) printf("[CartesianBot] Using tool: %d\n",tool);
    if (tool == 0) {
        //if (!isQuiet) printf("[CartesianBot] Using base coordinates.\n");
        targetX[0]=xd[0];
        targetX[1]=xd[1];
        targetX[2]=xd[2];
        targetO[0]=od[0];
        targetO[1]=od[1];
    } else if (tool == 1) {
        //if (!isQuiet) printf("[CartesianBot] Using robot tip coordinates.\n");
        yarp::sig::Matrix H_0_N = eulerYZtoH(x,o);
        yarp::sig::Matrix H_N_target = eulerYZtoH(xd,od);
        yarp::sig::Matrix H_0_target = H_0_N * H_N_target;
        targetX[0]=H_0_target(0,3);
        targetX[1]=H_0_target(1,3);
        targetX[2]=H_0_target(2,3);
        // https://truesculpt.googlecode.com/hg-history/38000e9dfece971460473d5788c235fbbe82f31b/Doc/rotation_matrix_to_euler.pdf
        // "Computing Euler angles from a rotation matrix", Gregory G. Slabaugh
        //targetO[0]=-toDeg(asin(H_0_target(2,0)));  // Alternatively: pi + asin(...)
        targetO[0]=o[0]+od[0];  // !!!! (but why not!?)
        targetO[1]=o[1]+od[1];  // !!!! (but why not!?)
    } else fprintf(stderr, "[CartesianBot] warning: Tool %d not implemented.\n",tool);
    //if (!isQuiet) printf("[CartesianBot] goToPose() Begin setting absolute base movement.\n");
    //if (!isQuiet) printf("[CartesianBot] goToPose() \\begin{Problem statement}\n");
    //if (!isQuiet) printf("[CartesianBot] x: %s \t o: %s\n",x.toString().c_str(),o.toString().c_str());
    //if (!isQuiet) printf("[CartesianBot] targetX: %s \t targetO: %s\n",targetX.toString().c_str(),targetO.toString().c_str());
    //if (!isQuiet)printf("[CartesianBot] goToPose() \\end{Problem statement}\n");
    double trajT=0.0;//duration;
    if (t>0) trajT = t;
    trajPrP = new OrderOneTraj;
    trajPrP->configure(sqrt(x[0]*x[0]+x[1]*x[1]),sqrt(targetX[0]*targetX[0]+targetX[1]*targetX[1]),trajT);
    trajPhP = new OrderOneTraj;
    trajPhP->configure(x[2]-A0,targetX[2]-A0,trajT);
    trajOyP = new OrderOneTraj;
    trajOyP->configure(o[0],targetO[0],trajT);  // We set it in degrees
    trajOz = new OrderOneTraj;
    trajOz->configure(toDeg(atan2(x[1],x[0])),toDeg(atan2(targetX[1],targetX[0])),trajT);
    trajOzPP = new OrderOneTraj;
    trajOzPP->configure(o[1],targetO[1],trajT);  // We set it in degrees
/*    printf("[goToPose] begin: trajPrP dump(100 samples).\n");
    trajPrP->dump(100);
    printf("[goToPose] end: trajPrP dump(100 samples).\n");
    printf("[goToPose] begin: trajPhP dump(100 samples).\n");
    trajPhP->dump(100);
    printf("[goToPose] end: trajPhP dump(100 samples).\n");
    printf("[goToPose] begin: trajOyP dump(100 samples).\n");
    trajOyP->dump(100);
    printf("[goToPose] end: trajOyP dump(100 samples).\n");
    printf("[goToPose] begin: trajOz dump(100 samples).\n");
    trajOz->dump(100);
    printf("[goToPose] end: trajOz dump(100 samples).\n");
    printf("[goToPose] begin: trajOzPP dump(100 samples).\n");
    trajOzPP->dump(100);
    printf("[goToPose] end: trajOzPP dump(100 samples).\n");*/
    startTime = yarp::os::Time::now();
    withOri=true;
    /*for (int i=0; i<NUM_MOTORS; i++)
        mode->setVelocityMode(i);
    cmc_status=1;*/
    //if (!isQuiet) printf("[CartesianBot] goToPose() End setting absolute base movement.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::goToPoseSync(const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                              const double t) {
    using namespace yarp::math;  // just for matrix operators
    yarp::sig::Vector x,o;  // empty vectors
    //getPose(x,o);  // where we put the result of performing fwd kinematics of current position
    //if (!isQuiet) printf("[CartesianBot] Using tool: %d\n",tool);
    if (tool == 0) {
        //if (!isQuiet) printf("[CartesianBot] Using base coordinates.\n");
        targetX[0]=xd[0];
        targetX[1]=xd[1];
        targetX[2]=xd[2];
        targetO[0]=od[0];
        targetO[1]=od[1];
    } else if (tool == 1) {
        //if (!isQuiet) printf("[CartesianBot] Using robot tip coordinates.\n");
        yarp::sig::Matrix H_0_N = eulerYZtoH(x,o);
        //if (!isQuiet) printf("H_0_N:\n%s\n\n",H_0_N.toString().c_str());
        yarp::sig::Matrix H_N_target = eulerYZtoH(xd,od);
        //if (!isQuiet) printf("H_N_target:\n%s\n\n",H_N_target.toString().c_str());
        yarp::sig::Matrix H_0_target = H_0_N * H_N_target;
        //if (!isQuiet) printf("H_0_target:\n%s\n\n",H_0_target.toString().c_str());
        targetX[0]=H_0_target(0,3);
        targetX[1]=H_0_target(1,3);
        targetX[2]=H_0_target(2,3);
        // https://truesculpt.googlecode.com/hg-history/38000e9dfece971460473d5788c235fbbe82f31b/Doc/rotation_matrix_to_euler.pdf
        // "Computing Euler angles from a rotation matrix", Gregory G. Slabaugh
        //targetO[0]=-toDeg(asin(H_0_target(2,0)));  // Alternatively: pi + asin(...)
        targetO[0]=o[0]+od[0];  // !!!! (but why not!?)
        targetO[1]=o[1]+od[1];  // !!!! (but why not!?)
    } else fprintf(stderr, "[CartesianBot] warning: Tool %d not implemented.\n",tool);
    //if (!isQuiet) printf("[CartesianBot] goToPose() Begin setting absolute base movement.\n");
    //if (!isQuiet) printf("[CartesianBot] goToPose() \\begin{Problem statement}\n");
    //if (!isQuiet) printf("[CartesianBot] x: %s \t o: %s\n",x.toString().c_str(),o.toString().c_str());
    //if (!isQuiet) printf("[CartesianBot] targetX: %s \t targetO: %s\n",targetX.toString().c_str(),targetO.toString().c_str());
    //if (!isQuiet)printf("[CartesianBot] goToPose() \\end{Problem statement}\n");
    double trajT=0.0;//duration;
    if (t>0) trajT = t;
    trajPrP = new OrderThreeTraj;
    trajPrP->configure(sqrt(x[0]*x[0]+x[1]*x[1]),sqrt(targetX[0]*targetX[0]+targetX[1]*targetX[1]),trajT);
    trajPhP = new OrderThreeTraj;
    trajPhP->configure(x[2]-A0,targetX[2]-A0,trajT);
    trajOyP = new OrderThreeTraj;
    trajOyP->configure(o[0],targetO[0],trajT);  // We set it in degrees
    trajOz = new OrderThreeTraj;
    trajOz->configure(toDeg(atan2(x[1],x[0])),toDeg(atan2(targetX[1],targetX[0])),trajT);
    trajOzPP = new OrderThreeTraj;
    trajOzPP->configure(o[1],targetO[1],trajT);  // We set it in degrees
/*    printf("[CartesianBot] goToPoseSync trajPrP dump(100 samples) begin.\n");
    trajPrP->dump(100);
    printf("[CartesianBot] goToPoseSync trajPrP dump(100 samples) end.\n");
    printf("[CartesianBot] goToPoseSync trajPhP dump(100 samples) begin.\n");
    trajPhP->dump(100);
    printf("[CartesianBot] goToPoseSync trajPhP dump(100 samples) end.\n");
    printf("[CartesianBot] goToPoseSync trajOyP dump(100 samples) begin.\n");
    trajOyP->dump(100);
    printf("[CartesianBot] goToPoseSync trajOyP dump(100 samples) end.\n");
    printf("[CartesianBot] goToPoseSync trajOz dump(100 samples) begin.\n");
    trajOz->dump(100);
    printf("[CartesianBot] goToPoseSync trajOz dump(100 samples) end.\n");
    printf("[CartesianBot] goToPoseSync trajOzPP dump(100 samples) begin.\n");
    trajOzPP->dump(100);
    printf("[CartesianBot] goToPoseSync trajOzPP dump(100 samples) end.\n");*/
    startTime = yarp::os::Time::now();
    withOri=true;
    /*for (int i=0; i<NUM_MOTORS; i++)
        mode->setVelocityMode(i);
    cmc_status=1;*/
    printf("[CartesianBot] End setting absolute base movement.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setTaskVelocities(const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot) {
    using namespace yarp::math;  // just for matrix operators
    double realDeg[NUM_MOTORS];  // Fixed because CartesianBot is very ASIBOT-specific
    /*if(!enc->getEncoders(realDeg)) {
        fprintf(stderr,"[CartesianBot] warning: setTaskVelocities() failed to getEncoders()\n");
        return false;  // bad practice??
    }*/
    yarp::sig::Matrix Ja(5,5);
    for (int i=0; i<NUM_MOTORS; i++)
        realRad[i]=toRad(realDeg[i]);
    Ja(0,0) = -sin(realRad[0])*(A2*sin(realRad[1] + realRad[2]) + A1*sin(realRad[1]) + A3*sin(realRad[1] + realRad[2] + realRad[3]));
    Ja(0,1) = cos(realRad[0])*(A2*cos(realRad[1] + realRad[2]) + A1*cos(realRad[1]) + A3*cos(realRad[1] + realRad[2] + realRad[3]));
    Ja(0,2) = cos(realRad[0])*(A2*cos(realRad[1] + realRad[2]) + A3*cos(realRad[1] + realRad[2] + realRad[3]));
    Ja(0,3) = A3*cos(realRad[1] + realRad[2] + realRad[3])*cos(realRad[0]);
    Ja(0,4) = 0;
    Ja(1,0) = cos(realRad[0])*(A2*sin(realRad[1] + realRad[2]) + A1*sin(realRad[1]) + A3*sin(realRad[1] + realRad[2] + realRad[3]));
    Ja(1,1) = sin(realRad[0])*(A2*cos(realRad[1] + realRad[2]) + A1*cos(realRad[1]) + A3*cos(realRad[1] + realRad[2] + realRad[3]));
    Ja(1,2) = sin(realRad[0])*(A2*cos(realRad[1] + realRad[2]) + A3*cos(realRad[1] + realRad[2] + realRad[3]));
    Ja(1,3) = A3*cos(realRad[1] + realRad[2] + realRad[3])*sin(realRad[0]);
    Ja(1,4) = 0;
    Ja(2,0) = 0;
    Ja(2,1) = - A2*sin(realRad[1] + realRad[2]) - A1*sin(realRad[1]) - A3*sin(realRad[1] + realRad[2] + realRad[3]);
    Ja(2,2) = - A2*sin(realRad[1] + realRad[2]) - A3*sin(realRad[1] + realRad[2] + realRad[3]);
    Ja(2,3) = -A3*sin(realRad[1] + realRad[2] + realRad[3]);
    Ja(2,4) = 0;
    Ja(3,0) = 0;
    Ja(3,1) = 1;
    Ja(3,2) = 1;
    Ja(3,3) = 1;
    Ja(3,4) = 0;
    Ja(4,0) = 0;
    Ja(4,1) = 0;
    Ja(4,2) = 0;
    Ja(4,3) = 0;
    Ja(4,4) = 1;
    yarp::sig::Matrix Ja_pinv(yarp::math::pinv(Ja,1.0e-2));
    //yarp::sig::Vector xdotd(xdot);
    yarp::sig::Vector xdotd;
    if (tool == 0) {
        xdotd.push_back(xdot[0]);
        xdotd.push_back(xdot[1]);
        xdotd.push_back(xdot[2]);
        xdotd.push_back(toRad(odot[0]));
        xdotd.push_back(toRad(odot[1]));
    } else if (tool == 1) {
        yarp::sig::Vector x,o;  // empty vectors
        //fwdKin(realDeg,x,o);  // Modifies x and o, returning success/fail value.
        yarp::sig::Matrix H_0_N = eulerYZtoH(x,o);
        H_0_N(0,3) = 0;
        H_0_N(1,3) = 0;
        H_0_N(2,3) = 0;
        //if (!isQuiet) printf("H_0_N (w/o pos):\n%s\n\n",H_0_N.toString().c_str());
        yarp::sig::Matrix H_N_target = eulerYZtoH(xdot,odot);
        //if (!isQuiet) printf("H_N_target:\n%s\n\n",H_N_target.toString().c_str());
        yarp::sig::Matrix H_0_target = H_0_N * H_N_target;
        //if (!isQuiet) printf("H_0_target:\n%s\n\n",H_0_target.toString().c_str());
        xdotd.push_back( H_0_target(0,3) );
        xdotd.push_back( H_0_target(1,3) );
        xdotd.push_back( H_0_target(2,3) );
        xdotd.push_back( toRad(odot[0]) ); //o[0]+od[0];  // !!!! (but why not!?)
        xdotd.push_back( toRad(odot[1]) ); //o[1]+od[1];  // !!!! (but why not!?)
    } else fprintf(stderr, "[CartesianBot] warning: Tool %d not implemented.\n",tool);
    
    yarp::sig::Vector t;
    t.resize(5);
    t = Ja_pinv * xdotd;
    double qdot[NUM_MOTORS];
    qdot[0] = toDeg(t[0]);
    qdot[1] = toDeg(t[1]);
    qdot[2] = toDeg(t[2]);
    qdot[3] = toDeg(t[3]);
    qdot[4] = toDeg(t[4]);
    /*for (int i=0; i<NUM_MOTORS; i++)
        mode->setVelocityMode(i);
    if(!vel->velocityMove(qdot))
        fprintf(stderr,"[CartesianBot] warning: COULD NOT SEND VELOCITY MOVE!!!\n");*/
    return true;
}

// -----------------------------------------------------------------------------

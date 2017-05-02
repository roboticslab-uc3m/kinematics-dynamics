// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <AsibotSolver.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setTrackingMode(const bool f) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getTrackingMode(bool *f) {
    return false;
}

// ----------------------------------------------------------------------------- 

bool roboticslab::AsibotSolver::getPose(const int axis, yarp::sig::Vector &x, yarp::sig::Vector &o, yarp::os::Stamp *stamp) {
    return false;
}

// ----------------------------------------------------------------------------- 

bool roboticslab::AsibotSolver::getPose(yarp::sig::Vector &x, yarp::sig::Vector &o, yarp::os::Stamp *stamp) {
    double realDeg[NUM_MOTORS];
    if(!enc->getEncoders(realDeg)) {
        fprintf(stderr,"[CartesianBot] warning: getPose() failed to getEncoders()\n");
        return false;
    }
    //return fwdKin(realDeg,x,o);  // Modifies x and o, returning success/fail value.
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::goToPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t) {
    yarp::sig::Vector x,o;  // empty vectors
    getPose(x,o);  // where we put the result of performing fwd kinematics of current position
    if (!isQuiet) printf("[CartesianBot] Using tool: %d\n",tool);
    if (tool == 0) {
        if (!isQuiet) printf("[CartesianBot] Using base coordinates.\n");
        targetX[0]=xd[0];
        targetX[1]=xd[1];
        targetX[2]=xd[2];
        targetO[0]=od[0];
        targetO[1]=od[1];
    } else if (tool == 1) {
        if (!isQuiet) printf("[CartesianBot] Using robot tip coordinates.\n");
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
    if (!isQuiet) printf("[CartesianBot] goToPose() Begin setting absolute base movement.\n");
    if (!isQuiet) printf("[CartesianBot] goToPose() \\begin{Problem statement}\n");
    if (!isQuiet) printf("[CartesianBot] x: %s \t o: %s\n",x.toString().c_str(),o.toString().c_str());
    if (!isQuiet) printf("[CartesianBot] targetX: %s \t targetO: %s\n",targetX.toString().c_str(),targetO.toString().c_str());
    if (!isQuiet)printf("[CartesianBot] goToPose() \\end{Problem statement}\n");
    double trajT=duration;
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
    startTime = Time::now();
    withOri=true;
    for (int i=0; i<NUM_MOTORS; i++)
        mode->setVelocityMode(i);
    cmc_status=1;
    if (!isQuiet) printf("[CartesianBot] goToPose() End setting absolute base movement.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::goToPosition(const yarp::sig::Vector &xd, const double t) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::goToPoseSync(const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                              const double t) {
    yarp::sig::Vector x,o;  // empty vectors
    getPose(x,o);  // where we put the result of performing fwd kinematics of current position
    if (!isQuiet) printf("[CartesianBot] Using tool: %d\n",tool);
    if (tool == 0) {
        if (!isQuiet) printf("[CartesianBot] Using base coordinates.\n");
        targetX[0]=xd[0];
        targetX[1]=xd[1];
        targetX[2]=xd[2];
        targetO[0]=od[0];
        targetO[1]=od[1];
    } else if (tool == 1) {
        if (!isQuiet) printf("[CartesianBot] Using robot tip coordinates.\n");
        yarp::sig::Matrix H_0_N = eulerYZtoH(x,o);
        if (!isQuiet) printf("H_0_N:\n%s\n\n",H_0_N.toString().c_str());
        yarp::sig::Matrix H_N_target = eulerYZtoH(xd,od);
        if (!isQuiet) printf("H_N_target:\n%s\n\n",H_N_target.toString().c_str());
        yarp::sig::Matrix H_0_target = H_0_N * H_N_target;
        if (!isQuiet) printf("H_0_target:\n%s\n\n",H_0_target.toString().c_str());
        targetX[0]=H_0_target(0,3);
        targetX[1]=H_0_target(1,3);
        targetX[2]=H_0_target(2,3);
        // https://truesculpt.googlecode.com/hg-history/38000e9dfece971460473d5788c235fbbe82f31b/Doc/rotation_matrix_to_euler.pdf
        // "Computing Euler angles from a rotation matrix", Gregory G. Slabaugh
        //targetO[0]=-toDeg(asin(H_0_target(2,0)));  // Alternatively: pi + asin(...)
        targetO[0]=o[0]+od[0];  // !!!! (but why not!?)
        targetO[1]=o[1]+od[1];  // !!!! (but why not!?)
    } else fprintf(stderr, "[CartesianBot] warning: Tool %d not implemented.\n",tool);
    if (!isQuiet) printf("[CartesianBot] goToPose() Begin setting absolute base movement.\n");
    if (!isQuiet) printf("[CartesianBot] goToPose() \\begin{Problem statement}\n");
    if (!isQuiet) printf("[CartesianBot] x: %s \t o: %s\n",x.toString().c_str(),o.toString().c_str());
    if (!isQuiet) printf("[CartesianBot] targetX: %s \t targetO: %s\n",targetX.toString().c_str(),targetO.toString().c_str());
    if (!isQuiet)printf("[CartesianBot] goToPose() \\end{Problem statement}\n");
    double trajT=duration;
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
    startTime = Time::now();
    withOri=true;
    for (int i=0; i<NUM_MOTORS; i++)
        mode->setVelocityMode(i);
    cmc_status=1;
    printf("[CartesianBot] End setting absolute base movement.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::goToPositionSync(const yarp::sig::Vector &xd, const double t) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getDesired(yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::askForPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                            yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat) {
    printf("Problem statement:\n");
    printf("xd: %s\nod: %s\n",xd.toString().c_str(),od.toString().c_str());
    double ozdRad = atan2(xd[1],xd[0]);
    double prPd = sqrt(xd[0]*xd[0]+xd[1]*xd[1]);
    double phPd = xd[2]-A0;
    double oyPd = od[0];
    printf("Problem statement:\n");
    printf("ozd: %f\nprPd: %f\nphPd: %f\n",toDeg(ozdRad),prPd,phPd);
    // t1=qdhat[1],t2=qdhat[2],t3=qdhat[3]
    double prWd = prPd - A3*sin(toRad(oyPd));
    double phWd = phPd - A3*cos(toRad(oyPd));
    double ct2 = (prWd*prWd + phWd*phWd - A1*A1 - A2*A2)/(2*A1*A2);
    double st2 = sqrt(1-ct2*ct2);  // forces elbow-up in ASIBOT
    //double st2 = -sqrt(1-ct2*ct2);  // forces elbow-down in ASIBOT
    printf("prWd: %f, phWd:%f\n",prWd,phWd);
    double t2Rad = atan2(st2,ct2);
    double st1 = ((A1+A2*ct2)*prWd - A2*st2*phWd)/(phWd*phWd+prWd*prWd);
    double ct1 = ((A1+A2*ct2)*phWd + A2*st2*prWd)/(phWd*phWd+prWd*prWd);
    // double ct1 = (phWd+A2*st1*st2)/(A1+A2*ct2);  // Alternative method for same result
    double t1Rad = atan2(st1,ct1);
    qdhat.resize(5);
    qdhat[0] = toDeg(ozdRad);
    qdhat[1] = toDeg(t1Rad);
    qdhat[2] = toDeg(t2Rad);
    qdhat[3] = oyPd - qdhat[1] - qdhat[2];
    qdhat[4] = od[1];  // ozPP
// Do the fwd kin for this and then:
    //xdhat.resize(3);
//    xdhat[0] = ;
    //odhat.resize(2);
//    odhat[0] = ;
    //fwdKin(qdhat,xdhat,odhat);  // 
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::askForPose(const yarp::sig::Vector &q0,
                            const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                            yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::askForPosition(const yarp::sig::Vector &xd,
                                yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                                yarp::sig::Vector &qdhat) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::askForPosition(const yarp::sig::Vector &q0,
                                const yarp::sig::Vector &xd,
                                yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                                yarp::sig::Vector &qdhat) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getDOF(yarp::sig::Vector &curDof) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setDOF(const yarp::sig::Vector &newDof, yarp::sig::Vector &curDof) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getRestPos(yarp::sig::Vector &curRestPos) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setRestPos(const yarp::sig::Vector &newRestPos, yarp::sig::Vector &curRestPos) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getRestWeights(yarp::sig::Vector &curRestWeights) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setRestWeights(const yarp::sig::Vector &newRestWeights, yarp::sig::Vector &curRestWeights) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getLimits(const int axis, double *min, double *max) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setLimits(const int axis, const double min, const double max) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getTrajTime(double *t) {
    *t = duration;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setTrajTime(const double t) {
    duration = t;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getInTargetTol(double *tol) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setInTargetTol(const double tol) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getJointsVelocities(yarp::sig::Vector &qdot) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getTaskVelocities(yarp::sig::Vector &xdot, yarp::sig::Vector &odot) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setTaskVelocities(const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot) {
    double realDeg[NUM_MOTORS];  // Fixed because CartesianBot is very ASIBOT-specific
    if(!enc->getEncoders(realDeg)) {
        fprintf(stderr,"[CartesianBot] warning: setTaskVelocities() failed to getEncoders()\n");
        return false;  // bad practice??
    }
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
    yarp::sig::Matrix Ja_pinv(pinv(Ja,1.0e-2));
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
        if (!isQuiet) printf("H_0_N (w/o pos):\n%s\n\n",H_0_N.toString().c_str());
        yarp::sig::Matrix H_N_target = eulerYZtoH(xdot,odot);
        if (!isQuiet) printf("H_N_target:\n%s\n\n",H_N_target.toString().c_str());
        yarp::sig::Matrix H_0_target = H_0_N * H_N_target;
        if (!isQuiet) printf("H_0_target:\n%s\n\n",H_0_target.toString().c_str());
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
    for (int i=0; i<NUM_MOTORS; i++)
        mode->setVelocityMode(i);
    if(!vel->velocityMove(qdot))
        fprintf(stderr,"[CartesianBot] warning: COULD NOT SEND VELOCITY MOVE!!!\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::attachTipFrame(const yarp::sig::Vector &x, const yarp::sig::Vector &o) {
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getTipFrame(yarp::sig::Vector &x, yarp::sig::Vector &o) {
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::removeTipFrame() {
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::checkMotionDone(bool *f) {
    bool tmpf = false;
    if(cmc_status<=0) tmpf = true;
    *f = tmpf;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::waitMotionDone(const double period, const double timeout) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::stopControl() {
    cmc_status=-1;
    printf("[CartesianBot] stopControl() End\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::storeContext(int *id) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::restoreContext(const int id) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getInfo(yarp::os::Bottle &info) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::registerEvent(yarp::dev::CartesianEvent &event) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::unregisterEvent(yarp::dev::CartesianEvent &event) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::setReferenceMode(const bool f) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::getReferenceMode(bool *f) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::tweakSet(const yarp::os::Bottle &options) {
    Bottle &opt=const_cast<Bottle&>(options);  // Ugo knows why... :-)
    if (!isQuiet) printf("[CartesianBot] tweakSet: %s\n", opt.toString().c_str());
    if (opt.check("tool")) tool = opt.find("tool").asInt();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::tweakGet(yarp::os::Bottle &options) {
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::deleteContext(const int id) {
    return false;
}

// -----------------------------------------------------------------------------


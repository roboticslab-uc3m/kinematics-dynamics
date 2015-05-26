
// -----------------------------------------------------------------------------

#include "KdlController.hpp"

// -----------------------------------------------------------------------------

bool teo::KdlController::setTrackingMode(const bool f) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getTrackingMode(bool *f) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setPosePriority(const yarp::os::ConstString &p) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getPosePriority(yarp::os::ConstString &p) {
    return false;
}

// ----------------------------------------------------------------------------- 

bool teo::KdlController::getPose(const int axis, yarp::sig::Vector &x, yarp::sig::Vector &o, yarp::os::Stamp *stamp) {
    return false;
}

// ----------------------------------------------------------------------------- 

bool teo::KdlController::getPose(yarp::sig::Vector &x, yarp::sig::Vector &o, yarp::os::Stamp *stamp) {
    double dRealUnits[100];
    if(!enc->getEncoders(dRealUnits)) {
        CD_WARNING("Failed to getEncoders()\n");
        return false;
    }
    yarp::sig::Vector realUnits(cmcNumMotors,dRealUnits);
    return fwdKin(realUnits,x,o);
}

// -----------------------------------------------------------------------------

bool teo::KdlController::goToPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::goToPosition(const yarp::sig::Vector &xd, const double t) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::goToPoseSync(const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                              const double t) {
    CD_INFO("Begin setting absolute base movement.\n");

    targetF.p.data[0] = xd[0];
    targetF.p.data[1] = xd[1];
    targetF.p.data[2] = xd[2];

    for (int s=0;s<od.size();s++)
        targetO[s]=od[s];

    if (angleRepr == "eulerYZ") {  // ASIBOT
        targetF.M = Rotation::EulerZYZ(::atan2(xd[1],xd[0]),toRad(od[0]), toRad(od[1]));
    } else if (angleRepr == "eulerZYZ") {
        targetF.M = Rotation::EulerZYZ(toRad(od[0]), toRad(od[1]), toRad(od[2]));
    } else if (angleRepr == "RPY") {
        targetF.M = Rotation::RPY(toRad(od[0]), toRad(od[1]), toRad(od[2]));
    } else {  // axisAngle, etc.
        CD_WARNING("Not compatible with angleRepr: %s.\n",angleRepr.c_str());
    }

    yarp::sig::Vector x,o;
    if( ! getPose(x,o) ) {
        CD_ERROR("GetPose failed.\n");
        return false;
    }

    KDL::Frame initF;
    initF.p.data[0] = x[0];
    initF.p.data[1] = x[1];
    initF.p.data[2] = x[2];
    if (angleRepr == "eulerYZ") {  // ASIBOT
        initF.M = Rotation::EulerZYZ(::atan2(x[1],x[0]),toRad(o[0]), toRad(o[1]));
    } else if (angleRepr == "eulerZYZ") {
        initF.M = Rotation::EulerZYZ(toRad(o[0]), toRad(o[1]), toRad(o[2]));
    } else if (angleRepr == "RPY") {
        initF.M = Rotation::RPY(toRad(o[0]), toRad(o[1]), toRad(o[2]));
    } else {  // axisAngle, etc.
        CD_WARNING("Not compatible with angleRepr: %s.\n",angleRepr.c_str());
    }

    double trajT=duration;
    if (t>0) trajT = t;

    _orient = new RotationalInterpolation_SingleAxis();
    double _eqradius = 1; //0.000001;
    bool _aggregate = true;

    KDL::Path_Line testPathLine(initF, targetF, _orient, _eqradius, _aggregate);
    KDL::VelocityProfile_Trap testVelocityProfile(maxVel, maxAcc);
//    KDL::Trajectory_Segment testTrajectory(testPathLine.Clone(), testVelocityProfile.Clone(), duration, _aggregate);
    currentTrajectory = new Trajectory_Segment(testPathLine.Clone(), testVelocityProfile.Clone(), duration, _aggregate);

    startTime = Time::now();
    withOri=true;
    vel->setVelocityMode();
    cmc_status=1;

    CD_INFO("End setting absolute base movement.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::goToPositionSync(const yarp::sig::Vector &xd, const double t) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getDesired(yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::askForPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                            yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat) {
    double currentUnits[100];
    if(!enc->getEncoders(currentUnits)) {
        CD_WARNING("Failed to getEncoders()\n");
        return false;
    }
    JntArray currentRads = JntArray(cmcNumMotors);
    for (int motor=0; motor<cmcNumMotors; motor++) {
        if(isPrismatic[motor]) currentRads(motor)=currentUnits[motor];
        else currentRads(motor)=toRad(currentUnits[motor]);
    }

    Frame frameXd;
    frameXd.p.data[0]=xd[0];
    frameXd.p.data[1]=xd[1];
    frameXd.p.data[2]=xd[2];
    if (angleRepr == "eulerYZ") {
        frameXd.M = Rotation::EulerZYZ(::atan2(xd[1],xd[0]),toRad(od[0]), toRad(od[1]));
    } else if (angleRepr == "eulerZYZ") {
        frameXd.M = Rotation::EulerZYZ(toRad(od[0]), toRad(od[1]), toRad(od[2]));
    } else if (angleRepr == "RPY") {
        frameXd.M = Rotation::RPY(toRad(od[0]), toRad(od[1]), toRad(od[2]));
    } else {
        CD_WARNING("Not compatible angleRepr: %s\n",angleRepr.c_str());
    }

    ChainFkSolverPos_recursive fksolver(theChain);
    ChainIkSolverVel_pinv iksolver_vel(theChain);
//    ChainIkSolverPos_NR iksolver_pos (theChain,fksolver,iksolver_vel,500,1e-6);
    JntArray qMin = JntArray(cmcNumMotors);
    JntArray qMax = JntArray(cmcNumMotors);
    for (int motor=0; motor<cmcNumMotors; motor++) {
        double _qMin, _qMax;
        if(!lim->getLimits(motor,&(_qMin),&(_qMax))) {
            CD_WARNING("getLimits failed.\n");
            return false;
        }
        if(isPrismatic[motor]) {
            qMin(motor)=_qMin;
            qMax(motor)=_qMax;
        } else {
            qMin(motor)=toRad(_qMin);
            qMax(motor)=toRad(_qMax);
        }
    }
    //ChainIkSolverPos_NR_JL iksolver_pos (theChain,qMin,qMax,fksolver,iksolver_vel,5000,1e-4);

    Eigen::Matrix<double,6,1> L;
    L(0)=1;L(1)=1;L(2)=1;
    L(3)=0;L(4)=0;L(5)=0;
    ChainIkSolverPos_LMA iksolver_pos(theChain,L);

    JntArray kdlqd = JntArray(cmcNumMotors);
    int ret = iksolver_pos.CartToJnt(currentRads,frameXd,kdlqd);

    if(ret<0) {
        CD_WARNING("KDL iksolver_pos bad return value (%d).\n",ret);
        return false;
    } else {
        CD_SUCCESS("KDL iksolver_pos good return value (%d).\n",ret);
    }

    /*for (int motor=0; motor<cmcNumMotors; motor++) {
        if(isPrismatic[motor]) qdhat.push_back(qd(motor));
        else qdhat.push_back(toDeg(qd(motor)));
    }*/
    qdhat.resize(cmcNumMotors,0.0);
    for (int idxIdx=0; idxIdx<vectorOfCmcMotorIdxs.size(); idxIdx++) {
        int motor = vectorOfCmcMotorIdxs[idxIdx];
        if(isPrismatic[motor]) qdhat[motor]=kdlqd(idxIdx);
        else qdhat[motor]=toDeg(kdlqd(idxIdx));
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::askForPose(const yarp::sig::Vector &q0,
                            const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                            yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::askForPosition(const yarp::sig::Vector &xd,
                                yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                                yarp::sig::Vector &qdhat) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::askForPosition(const yarp::sig::Vector &q0,
                                const yarp::sig::Vector &xd,
                                yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                                yarp::sig::Vector &qdhat) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getDOF(yarp::sig::Vector &curDof) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setDOF(const yarp::sig::Vector &newDof, yarp::sig::Vector &curDof) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getRestPos(yarp::sig::Vector &curRestPos) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setRestPos(const yarp::sig::Vector &newRestPos, yarp::sig::Vector &curRestPos) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getRestWeights(yarp::sig::Vector &curRestWeights) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setRestWeights(const yarp::sig::Vector &newRestWeights, yarp::sig::Vector &curRestWeights) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getLimits(const int axis, double *min, double *max) {
    if(!lim->getLimits(axis,min,max)) return false;  // should pass to JMC
    printf("Range of axis %d is: %f to %f.\n",axis,*min,*max);
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setLimits(const int axis, const double min, const double max) {
    if(!lim->setLimits(axis,min,max)) return false;  // should pass to JMC
    printf("[KdlControllerRange of axis %d set to: %f to %f on JMC\n",axis,min,max);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getTrajTime(double *t) {
    *t = duration;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setTrajTime(const double t) {
    duration = t;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getInTargetTol(double *tol) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setInTargetTol(const double tol) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getJointsVelocities(yarp::sig::Vector &qdot) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getTaskVelocities(yarp::sig::Vector &xdot, yarp::sig::Vector &odot) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setTaskVelocities(const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::attachTipFrame(const yarp::sig::Vector &x, const yarp::sig::Vector &o) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getTipFrame(yarp::sig::Vector &x, yarp::sig::Vector &o) {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::removeTipFrame() {
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::checkMotionDone(bool *f) {
    bool tmpf = false;
    if(cmc_status<=0) tmpf = true;
    *f = tmpf;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::waitMotionDone(const double period, const double timeout) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::stopControl() {
    cmc_status=-1;
    printf("[end] KdlController::stopControl()\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::storeContext(int *id) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::restoreContext(const int id) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getInfo(yarp::os::Bottle &info) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::registerEvent(yarp::dev::CartesianEvent &event) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::unregisterEvent(yarp::dev::CartesianEvent &event) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::setReferenceMode(const bool f) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::getReferenceMode(bool *f) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::tweakSet(const yarp::os::Bottle &options) {
    CD_INFO("%s\n", options.toString().c_str());
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::tweakGet(yarp::os::Bottle &options) {
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlController::deleteContext(const int id) {
    return false;
}

// -----------------------------------------------------------------------------


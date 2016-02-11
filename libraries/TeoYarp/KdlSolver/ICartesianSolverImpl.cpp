// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

// -----------------------------------------------------------------------------

bool teo::KdlSolver::getNumLinks(int* numLinks) {
    *numLinks = this->numLinks;
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x) {

    KDL::JntArray qInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qInRad(motor)=toRad(q[motor]);

    //-- Main fwdKin (pos) solver lines
    KDL::Frame fOutCart;
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    fksolver.JntToCart(qInRad,fOutCart);

    frameToVector(fOutCart,x);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::fwdKinError(const std::vector<double> &xd, const std::vector<double> &q, std::vector<double> &x) {

    KDL::Frame frameXd;
    vectorToFrame(xd,frameXd);

    KDL::JntArray qInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qInRad(motor)=toRad(q[motor]);

    //-- Main fwdKin (pos) solver lines
    KDL::Frame f;
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    fksolver.JntToCart(qInRad,f);

    KDL::Twist d = KDL::diff(frameXd,f);
    x.resize(6);
    x[0] = d.vel.x();
    x[1] = d.vel.y();
    x[2] = d.vel.z();
    x[3] = d.rot.x();
    x[4] = d.vel.y();
    x[5] = d.vel.z();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q) {

    KDL::Frame frameXd;
    vectorToFrame(xd,frameXd);

    Eigen::Matrix<double,6,1> L;
    L(0)=1;L(1)=1;L(2)=1;
    L(3)=0;L(4)=0;L(5)=0;

    KDL::JntArray qGuessInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qGuessInRad(motor)=toRad(qGuess[motor]);

    //-- Main invKin (pos) solver lines
    KDL::ChainIkSolverPos_LMA iksolver_pos(chain,L);
    KDL::JntArray kdlq = KDL::JntArray(numLinks);
    int ret = iksolver_pos.CartToJnt(qGuessInRad,frameXd,kdlq);

    q.resize(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        q[motor]=toDeg(kdlq(motor));

    if (ret == 0) return true;

    if (ret == -1)
    {
        CD_WARNING("The gradient of E towards the joints is to small.\n");
    }
    else if (ret == -2)
    {
        CD_WARNING("The joint position increments are to small.\n");
    }
    else if (ret == -3)
    {
        CD_WARNING("The number of iterations is exceeded.\n");
    }
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot) {

    KDL::JntArray qInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qInRad(motor)=toRad(q[motor]);

    KDL::Twist kdlxdot;
    kdlxdot.vel.x(xdot[0]);
    kdlxdot.vel.y(xdot[1]);
    kdlxdot.vel.z(xdot[2]);
    kdlxdot.rot.x(xdot[3]);
    kdlxdot.rot.y(xdot[4]);
    kdlxdot.rot.z(xdot[5]);

    KDL::JntArray qDotOutRadS = KDL::JntArray(numLinks);
    KDL::ChainIkSolverVel_pinv iksolverv(chain);
    int ret = iksolverv.CartToJnt(qInRad,kdlxdot,qDotOutRadS);

    qdot.resize(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qdot[motor]=toDeg(qDotOutRadS(motor));

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::invDyn(const std::vector<double> &q,std::vector<double> &t) {

    KDL::JntArray qInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qInRad(motor)=toRad(q[motor]);

    KDL::JntArray qdotInRad = KDL::JntArray(numLinks);
    qdotInRad.data.setZero();

    KDL::JntArray qdotdotInRad = KDL::JntArray(numLinks);
    qdotdotInRad.data.setZero();

    KDL::Wrenches wrenches(numLinks,KDL::Wrench::Zero());

    KDL::JntArray kdlt = KDL::JntArray(numLinks);

    //-- Main invDyn solver lines
    KDL::ChainIdSolver_RNE idsolver(chain,gravity);
    int ret = idsolver.CartToJnt(qInRad,qdotInRad,qdotdotInRad,wrenches,kdlt);

    if(ret < 1)
    {
        CD_ERROR("%s\n",idsolver.strError(ret));
        return false;
    }

    if(ret > 1)
        CD_WARNING("%s\n",idsolver.strError(ret));

    t.resize(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        t[motor]=kdlt(motor);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t) {

    KDL::JntArray qInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qInRad(motor)=toRad(q[motor]);

    KDL::JntArray qdotInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qdotInRad(motor)=toRad(qdot[motor]);

    KDL::JntArray qdotdotInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qdotdotInRad(motor)=toRad(qdotdot[motor]);

    KDL::Wrenches wrenches;
    for (int i=0; i<numLinks; i++)
    {
        KDL::Wrench wrench( KDL::Vector(fexts[i][0],fexts[i][1],fexts[i][2]), KDL::Vector(fexts[i][3],fexts[i][4],fexts[i][5]) );
        wrenches.push_back(wrench);
    }

    KDL::JntArray kdlt = KDL::JntArray(numLinks);

    //-- Main invDyn solver lines
    KDL::ChainIdSolver_RNE idsolver(chain,gravity);
    int ret = idsolver.CartToJnt(qInRad,qdotInRad,qdotdotInRad,wrenches,kdlt);

    if(ret < 1)
    {
        CD_ERROR("%s\n",idsolver.strError(ret));
        return false;
    }

    if(ret > 1)
        CD_WARNING("%s\n",idsolver.strError(ret));

    t.resize(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        t[motor]=kdlt(motor);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::vectorToFrame(const std::vector<double> &x, KDL::Frame& f) {

    f.p.data[0]=x[0];
    f.p.data[1]=x[1];
    f.p.data[2]=x[2];

    if (angleRepr == "axisAngle") {
        f.M = KDL::Rotation::Rot(KDL::Vector(x[3],x[4],x[5]),x[6]);
    }
    else if (angleRepr == "eulerYZ")  //-- like ASIBOT
    {
        f.M = KDL::Rotation::EulerZYZ(::atan2(x[1],x[0]),toRad(x[3]), toRad(x[4]));
    }
    else if (angleRepr == "eulerZYZ")
    {
        f.M = KDL::Rotation::EulerZYZ(toRad(x[3]), toRad(x[4]), toRad(x[5]));
    }
    else if (angleRepr == "RPY")
    {
        f.M = KDL::Rotation::RPY(toRad(x[3]), toRad(x[4]), toRad(x[5]));
    }
    else  //-- No known angle repr.
    {
        CD_WARNING("angleRepr unknown %s\n",angleRepr.c_str());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::frameToVector(const KDL::Frame& f, std::vector<double> &x) {

    //-- Fill angle first, then 0-2 for position.

    if (angleRepr == "axisAngle")
    {
        KDL::Vector rotVector = f.M.GetRot();
        x.resize(7);
        x[6] = f.M.GetRotAngle(rotVector);  // Normalizes as colateral effect
        x[3] = rotVector[0];
        x[4] = rotVector[1];
        x[5] = rotVector[2];
    }
    else if (angleRepr == "eulerYZ") //-- like ASIBOT
    {
        double alfa, beta, gamma;
        f.M.GetEulerZYZ(alfa, beta, gamma);
        x.resize(5);
        x[3] = toDeg(beta);
        x[4] = toDeg(gamma);
    }
    else if (angleRepr == "eulerZYZ")
    {
        double alfa, beta, gamma;
        f.M.GetEulerZYZ(alfa, beta, gamma);
        x.resize(6);
        x[3] = toDeg(alfa);
        x[4] = toDeg(beta);
        x[5] = toDeg(gamma);
    }
    else if (angleRepr == "RPY")
    {
        double alfa, beta, gamma;
        f.M.GetRPY(alfa, beta, gamma);
        x.resize(6);
        x[3] = toDeg(alfa);
        x[4] = toDeg(beta);
        x[5] = toDeg(gamma);
    }
    else  //-- No known angle repr.
    {
        CD_WARNING("angleRepr unknown %s\n",angleRepr.c_str());
    }

    x[0] = f.p.data[0];
    x[1] = f.p.data[1];
    x[2] = f.p.data[2];

    return true;
}

// -----------------------------------------------------------------------------

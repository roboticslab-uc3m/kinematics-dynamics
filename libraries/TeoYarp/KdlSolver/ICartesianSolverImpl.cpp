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
    x[4] = d.rot.y();
    x[5] = d.rot.z();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q) {

    KDL::Frame frameXd;
    vectorToFrame(xd,frameXd);

    KDL::JntArray qGuessInRad = KDL::JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qGuessInRad(motor)=toRad(qGuess[motor]);
    KDL::JntArray kdlq = KDL::JntArray(numLinks);

#ifdef _USE_LMA_

    Eigen::Matrix<double,6,1> L;
    L(0)=1;L(1)=1;L(2)=1;
    L(3)=0;L(4)=0;L(5)=0;


    //-- Main invKin (pos) solver lines
    KDL::ChainIkSolverPos_LMA iksolver_pos(chain,L);

#else //_USE_LMA_

    //Forward solvers, needed by the geometric solver
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    KDL::ChainIkSolverVel_pinv iksolver(chain);  // _givens

    //Geometric solver definition (with joint limits)
    KDL::ChainIkSolverPos_NR_JL iksolver_pos(chain,qMin,qMax,fksolver,iksolver,100000,1E-15);

#endif //_USE_LMA_

    int ret = iksolver_pos.CartToJnt(qGuessInRad,frameXd,kdlq);

    if(ret < 0)
    {
        CD_ERROR("%d: %s\n",ret,iksolver_pos.strError(ret));
        return false;
    }

    if(ret > 0)
        CD_WARNING("%d: %s\n",ret, iksolver_pos.strError(ret));

    q.resize(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        q[motor]=toDeg(kdlq(motor));

    return true;
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

    if(ret < 0)
    {
        CD_ERROR("%d: %s\n",ret,iksolverv.strError(ret));
        return false;
    }

    if(ret > 0)
        CD_WARNING("%d: %s\n",ret, iksolverv.strError(ret));

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

    if(ret < 0)
    {
        CD_ERROR("%d: %s\n",ret,idsolver.strError(ret));
        return false;
    }

    if(ret > 0)
        CD_WARNING("%d: %s\n",ret, idsolver.strError(ret));

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

    if(ret < 0)
    {
        CD_ERROR("%d: %s\n",ret,idsolver.strError(ret));
        return false;
    }

    if(ret > 0)
        CD_WARNING("%d: %s\n",ret,idsolver.strError(ret));

    t.resize(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        t[motor]=kdlt(motor);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::setLimits(const std::vector<double> &qMin, const std::vector<double> &qMax)
{
    for (int motor=0; motor<numLinks; motor++)
    {
        this->qMax(motor) = toRad(qMax[motor]);
        this->qMin(motor) = toRad(qMin[motor]);
    }
    return true;
}

// -----------------------------------------------------------------------------

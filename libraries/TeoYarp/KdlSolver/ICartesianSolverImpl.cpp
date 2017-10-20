// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

#include <kdl/segment.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <ColorDebug.hpp>

#include "KdlVectorConverter.hpp"
#include "KinematicRepresentation.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::getNumJoints(int* numJoints) {
    *numJoints = this->chain.getNrOfJoints();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::appendLink(const std::vector<double>& x) {
    KDL::Frame frameX = KdlVectorConverter::vectorToFrame(x);
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), frameX));
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::restoreOriginalChain() {
    chain = originalChain;  // We have: Chain& operator = (const Chain& arg);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x) {

    KDL::JntArray qInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);

    //-- Main fwdKin (pos) solver lines
    KDL::Frame fOutCart;
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    fksolver.JntToCart(qInRad,fOutCart);

    x = KdlVectorConverter::frameToVector(fOutCart);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::fwdKinError(const std::vector<double> &xd, const std::vector<double> &q, std::vector<double> &x) {

    KDL::Frame frameXd = KdlVectorConverter::vectorToFrame(xd);

    KDL::JntArray qInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);

    //-- Main fwdKin (pos) solver lines
    KDL::Frame f;
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    fksolver.JntToCart(qInRad,f);

    KDL::Twist d = KDL::diff(f,frameXd);
    x = KdlVectorConverter::twistToVector(d);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q) {

    KDL::Frame frameXd = KdlVectorConverter::vectorToFrame(xd);

    KDL::JntArray qGuessInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qGuessInRad(motor) = KinRepresentation::degToRad(qGuess[motor]);
    KDL::JntArray kdlq = KDL::JntArray(chain.getNrOfJoints());

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
    KDL::ChainIkSolverPos_NR_JL iksolver_pos(chain,qMin,qMax,fksolver,iksolver,maxIter,eps);

#endif //_USE_LMA_

    int ret = iksolver_pos.CartToJnt(qGuessInRad,frameXd,kdlq);

    if(ret < 0)
    {
        CD_ERROR("%d: %s\n",ret,iksolver_pos.strError(ret));
        return false;
    }

    if(ret > 0)
        CD_WARNING("%d: %s\n",ret, iksolver_pos.strError(ret));

    q.resize(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        q[motor] = KinRepresentation::radToDeg(kdlq(motor));

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot) {

    KDL::JntArray qInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);

    KDL::Twist kdlxdot = KdlVectorConverter::vectorToTwist(xdot);
    KDL::JntArray qDotOutRadS = KDL::JntArray(chain.getNrOfJoints());
    KDL::ChainIkSolverVel_pinv iksolverv(chain);
    int ret = iksolverv.CartToJnt(qInRad,kdlxdot,qDotOutRadS);

    if(ret < 0)
    {
        CD_ERROR("%d: %s\n",ret,iksolverv.strError(ret));
        return false;
    }

    if(ret > 0)
        CD_WARNING("%d: %s\n",ret, iksolverv.strError(ret));

    qdot.resize(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qdot[motor] = KinRepresentation::radToDeg(qDotOutRadS(motor));

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::diffInvKinEE(const std::vector<double> &q, const std::vector<double> &xdotee, std::vector<double> &qdot) {

    KDL::JntArray qInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);

    KDL::Twist kdlxdotee = KdlVectorConverter::vectorToTwist(xdotee);

    KDL::Frame fOutCart;
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    fksolver.JntToCart(qInRad,fOutCart);

    // transform the basis to which the twist is expressed, but leave the reference point intact
    // "Twist and Wrench transformations" @ http://docs.ros.org/latest/api/orocos_kdl/html/geomprim.html
    KDL::Twist kdlxdot = fOutCart.M * kdlxdotee;

    KDL::JntArray qDotOutRadS = KDL::JntArray(chain.getNrOfJoints());
    KDL::ChainIkSolverVel_pinv iksolverv(chain);
    int ret = iksolverv.CartToJnt(qInRad,kdlxdot,qDotOutRadS);

    if(ret < 0)
    {
        CD_ERROR("%d: %s\n",ret,iksolverv.strError(ret));
        return false;
    }

    if(ret > 0)
        CD_WARNING("%d: %s\n",ret, iksolverv.strError(ret));

    qdot.resize(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qdot[motor] = KinRepresentation::radToDeg(qDotOutRadS(motor));

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invDyn(const std::vector<double> &q,std::vector<double> &t) {

    KDL::JntArray qInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);

    KDL::JntArray qdotInRad = KDL::JntArray(chain.getNrOfJoints());
    qdotInRad.data.setZero();

    KDL::JntArray qdotdotInRad = KDL::JntArray(chain.getNrOfJoints());
    qdotdotInRad.data.setZero();

    KDL::Wrenches wrenches(chain.getNrOfSegments(),KDL::Wrench::Zero());

    KDL::JntArray kdlt = KDL::JntArray(chain.getNrOfJoints());

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

    t.resize(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        t[motor]=kdlt(motor);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t) {

    KDL::JntArray qInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);

    KDL::JntArray qdotInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qdotInRad(motor) = KinRepresentation::degToRad(qdot[motor]);

    KDL::JntArray qdotdotInRad = KDL::JntArray(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        qdotdotInRad(motor) = KinRepresentation::degToRad(qdotdot[motor]);

    KDL::Wrenches wrenches = KDL::Wrenches(chain.getNrOfSegments(),KDL::Wrench::Zero());
    for (int i=0; i<fexts.size(); i++)
    {
        KDL::Wrench wrench( KDL::Vector(fexts[i][0],fexts[i][1],fexts[i][2]), KDL::Vector(fexts[i][3],fexts[i][4],fexts[i][5]) );
        wrenches[i] = wrench;
    }

    KDL::JntArray kdlt = KDL::JntArray(chain.getNrOfJoints());

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

    t.resize(chain.getNrOfJoints());
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
        t[motor]=kdlt(motor);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::setLimits(const std::vector<double> &qMin, const std::vector<double> &qMax)
{
    for (int motor=0; motor<chain.getNrOfJoints(); motor++)
    {
        this->qMax(motor) = KinRepresentation::degToRad(qMax[motor]);
        this->qMin(motor) = KinRepresentation::degToRad(qMin[motor]);
    }
    return true;
}

// -----------------------------------------------------------------------------

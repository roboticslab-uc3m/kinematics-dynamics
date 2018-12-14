// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

#include <ColorDebug.h>

#include "KdlVectorConverter.hpp"
#include "KinematicRepresentation.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::getNumJoints(int* numJoints)
{
    *numJoints = chain.getNrOfJoints();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::appendLink(const std::vector<double>& x)
{
    KDL::Frame frameX = KdlVectorConverter::vectorToFrame(x);

    mutex.wait();

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), frameX));

    fkSolverPos->updateInternalDataStructures();
    ikSolverPos->updateInternalDataStructures();
    ikSolverVel->updateInternalDataStructures();
    idSolver->updateInternalDataStructures();

    mutex.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::restoreOriginalChain()
{
    mutex.wait();

    chain = originalChain;

    fkSolverPos->updateInternalDataStructures();
    ikSolverPos->updateInternalDataStructures();
    ikSolverVel->updateInternalDataStructures();
    idSolver->updateInternalDataStructures();

    mutex.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old,
        std::vector<double> &x_new_obj)
{
    KDL::Frame H_old_obj = KdlVectorConverter::vectorToFrame(x_old_obj);
    KDL::Frame H_new_old = KdlVectorConverter::vectorToFrame(x_new_old);
    KDL::Frame H_new_obj = H_new_old * H_old_obj;

    x_new_obj = KdlVectorConverter::frameToVector(H_new_obj);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    mutex.wait();

    KDL::Frame fOutCart;
    fkSolverPos->JntToCart(qInRad, fOutCart);

    mutex.post();

    x = KdlVectorConverter::frameToVector(fOutCart);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut)
{
    KDL::Frame fLhs = KdlVectorConverter::vectorToFrame(xLhs);
    KDL::Frame fRhs = KdlVectorConverter::vectorToFrame(xRhs);

    KDL::Twist diff = KDL::diff(fRhs, fLhs); // [fLhs - fRhs] for translation
    xOut = KdlVectorConverter::twistToVector(diff);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q,
        const reference_frame frame)
{
    KDL::Frame frameXd = KdlVectorConverter::vectorToFrame(xd);
    KDL::JntArray qGuessInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qGuessInRad(motor) = KinRepresentation::degToRad(qGuess[motor]);
    }

    KDL::JntArray kdlq(chain.getNrOfJoints());

    mutex.wait();

    if (frame == TCP_FRAME)
    {
        KDL::Frame fOutCart;
        fkSolverPos->JntToCart(qGuessInRad, fOutCart);
        frameXd = fOutCart * frameXd;
    }
    else if (frame != BASE_FRAME)
    {
        CD_WARNING("Unsupported frame.\n");
        mutex.post();
        return false;
    }

    int ret = ikSolverPos->CartToJnt(qGuessInRad, frameXd, kdlq);

    mutex.post();

    if (ret < 0)
    {
        CD_ERROR("%d: %s\n", ret, ikSolverPos->strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        CD_WARNING("%d: %s\n", ret, ikSolverPos->strError(ret));
    }

    q.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        q[motor] = KinRepresentation::radToDeg(kdlq(motor));
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot,
        const reference_frame frame)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    KDL::Twist kdlxdot = KdlVectorConverter::vectorToTwist(xdot);

    mutex.wait();

    if (frame == TCP_FRAME)
    {
        KDL::Frame fOutCart;
        fkSolverPos->JntToCart(qInRad, fOutCart);

        //-- Transform the basis to which the twist is expressed, but leave the reference point intact
        //-- "Twist and Wrench transformations" @ http://docs.ros.org/latest/api/orocos_kdl/html/geomprim.html
        kdlxdot = fOutCart.M * kdlxdot;
    }
    else if (frame != BASE_FRAME)
    {
        CD_WARNING("Unsupported frame.\n");
        mutex.post();
        return false;
    }

    KDL::JntArray qDotOutRadS(chain.getNrOfJoints());

    int ret = ikSolverVel->CartToJnt(qInRad, kdlxdot, qDotOutRadS);

    mutex.post();

    if (ret < 0)
    {
        CD_ERROR("%d: %s\n", ret, ikSolverVel->strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        CD_WARNING("%d: %s\n", ret, ikSolverVel->strError(ret));
    }

    qdot.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdot[motor] = KinRepresentation::radToDeg(qDotOutRadS(motor));
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invDyn(const std::vector<double> &q,std::vector<double> &t)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    KDL::JntArray qdotInRad(chain.getNrOfJoints());
    KDL::JntArray qdotdotInRad(chain.getNrOfJoints());

    mutex.wait();

    KDL::Wrenches wrenches(chain.getNrOfSegments(), KDL::Wrench::Zero());
    KDL::JntArray kdlt(chain.getNrOfJoints());

    int ret = idSolver->CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt);

    mutex.post();

    if (ret < 0)
    {
        CD_ERROR("%d: %s\n", ret, idSolver->strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        CD_WARNING("%d: %s\n", ret, idSolver->strError(ret));
    }

    t.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = KinRepresentation::degToRad(q[motor]);
    }

    KDL::JntArray qdotInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdotInRad(motor) = KinRepresentation::degToRad(qdot[motor]);
    }

    KDL::JntArray qdotdotInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdotdotInRad(motor) = KinRepresentation::degToRad(qdotdot[motor]);
    }

    mutex.wait();

    KDL::Wrenches wrenches(chain.getNrOfSegments(), KDL::Wrench::Zero());

    for (int i = 0; i < fexts.size(); i++)
    {
        wrenches[i] = KDL::Wrench(
            KDL::Vector(fexts[i][0], fexts[i][1], fexts[i][2]),
            KDL::Vector(fexts[i][3], fexts[i][4], fexts[i][5])
        );
    }

    KDL::JntArray kdlt(chain.getNrOfJoints());

    int ret = idSolver->CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt);

    mutex.post();

    if (ret < 0)
    {
        CD_ERROR("%d: %s\n", ret, idSolver->strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        CD_WARNING("%d: %s\n", ret, idSolver->strError(ret));
    }

    t.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return true;
}

// -----------------------------------------------------------------------------

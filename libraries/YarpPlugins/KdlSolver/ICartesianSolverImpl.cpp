// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/utilities/utility.h> // KDL::deg2rad, KDL::rad2deg

#include <yarp/os/Log.h>

#include "KdlVectorConverter.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

int KdlSolver::getNumJoints()
{
    return chain.getNrOfJoints();
}

// -----------------------------------------------------------------------------

int KdlSolver::getNumTcps()
{
    return 1;
}

// -----------------------------------------------------------------------------

bool KdlSolver::appendLink(const std::vector<double>& x)
{
    KDL::Frame frameX = KdlVectorConverter::vectorToFrame(x);

    std::lock_guard<std::mutex> lock(mtx);

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), frameX));

    fkSolverPos->updateInternalDataStructures();
    ikSolverPos->updateInternalDataStructures();
    ikSolverVel->updateInternalDataStructures();
    idSolver->updateInternalDataStructures();

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::restoreOriginalChain()
{
    std::lock_guard<std::mutex> lock(mtx);

    chain = originalChain;

    fkSolverPos->updateInternalDataStructures();
    ikSolverPos->updateInternalDataStructures();
    ikSolverVel->updateInternalDataStructures();
    idSolver->updateInternalDataStructures();

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old, std::vector<double> &x_new_obj)
{
    KDL::Frame H_old_obj = KdlVectorConverter::vectorToFrame(x_old_obj);
    KDL::Frame H_new_old = KdlVectorConverter::vectorToFrame(x_new_old);
    KDL::Frame H_new_obj = H_new_old * H_old_obj;

    x_new_obj = KdlVectorConverter::frameToVector(H_new_obj);

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    KDL::Frame fOutCart;

    {
        std::lock_guard<std::mutex> lock(mtx);
        fkSolverPos->JntToCart(qInRad, fOutCart);
    }

    x = KdlVectorConverter::frameToVector(fOutCart);

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut)
{
    KDL::Frame fLhs = KdlVectorConverter::vectorToFrame(xLhs);
    KDL::Frame fRhs = KdlVectorConverter::vectorToFrame(xRhs);

    KDL::Twist diff = KDL::diff(fRhs, fLhs); // [fLhs - fRhs] for translation
    xOut = KdlVectorConverter::twistToVector(diff);

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, const reference_frame frame)
{
    KDL::Frame frameXd = KdlVectorConverter::vectorToFrame(xd);
    KDL::JntArray qGuessInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qGuessInRad(motor) = qGuess[motor] * KDL::deg2rad;
    }

    KDL::JntArray kdlq(chain.getNrOfJoints());
    int ret;

    {
        std::lock_guard<std::mutex> lock(mtx);

        if (frame == TCP_FRAME)
        {
            KDL::Frame fOutCart;
            fkSolverPos->JntToCart(qGuessInRad, fOutCart);
            frameXd = fOutCart * frameXd;
        }
        else if (frame != BASE_FRAME)
        {
            yCWarning(logc, "Unsupported frame");
            return false;
        }

        ret = ikSolverPos->CartToJnt(qGuessInRad, frameXd, kdlq);
    }

    if (ret < 0)
    {
        yCError(logc, "invKin(): %s", ikSolverPos->strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        yCWarning(logc, "invKin(): %s", ikSolverPos->strError(ret));
    }

    q.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        q[motor] = kdlq(motor) * KDL::rad2deg;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, const reference_frame frame)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    KDL::Twist kdlxdot = KdlVectorConverter::vectorToTwist(xdot);
    KDL::JntArray qDotOutRadS(chain.getNrOfJoints());
    int ret;

    {
        std::lock_guard<std::mutex> lock(mtx);

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
            yCWarning(logc, "Unsupported frame");
            return false;
        }

        ret = ikSolverVel->CartToJnt(qInRad, kdlxdot, qDotOutRadS);
    }

    if (ret < 0)
    {
        yCError(logc, "diffInvKin(): %s", ikSolverVel->strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        yCWarning(logc, "diffInvKin(): %s", ikSolverVel->strError(ret));
    }

    qdot.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdot[motor] = qDotOutRadS(motor) * KDL::rad2deg;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::invDyn(const std::vector<double> &q,std::vector<double> &t)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    KDL::JntArray qdotInRad(chain.getNrOfJoints());
    KDL::JntArray qdotdotInRad(chain.getNrOfJoints());
    KDL::JntArray kdlt(chain.getNrOfJoints());
    KDL::Wrenches wrenches(chain.getNrOfSegments(), KDL::Wrench::Zero());

    int ret;

    {
        std::lock_guard<std::mutex> lock(mtx);
        ret = idSolver->CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt);
    }

    if (ret < 0)
    {
        yCError(logc, "invDyn(): %s", idSolver->strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        yCWarning(logc, "invDyn(): %s", idSolver->strError(ret));
    }

    t.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    KDL::JntArray qdotInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdotInRad(motor) = qdot[motor] * KDL::deg2rad;
    }

    KDL::JntArray qdotdotInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdotdotInRad(motor) = qdotdot[motor] * KDL::deg2rad;
    }

    KDL::Wrenches wrenches(chain.getNrOfSegments(), KDL::Wrench::Zero());

    for (int i = 0; i < fexts.size(); i++)
    {
        wrenches[i] = KDL::Wrench(
            KDL::Vector(fexts[i][0], fexts[i][1], fexts[i][2]),
            KDL::Vector(fexts[i][3], fexts[i][4], fexts[i][5])
        );
    }

    KDL::JntArray kdlt(chain.getNrOfJoints());
    int ret;

    {
        std::lock_guard<std::mutex> lock(mtx);
        ret = idSolver->CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt);
    }

    if (ret < 0)
    {
        yCError(logc, "invDyn(): %s", idSolver->strError(ret));
        return false;
    }
    else if (ret > 0)
    {
        yCWarning(logc, "invDyn(): %s", idSolver->strError(ret));
    }

    t.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return true;
}

// -----------------------------------------------------------------------------

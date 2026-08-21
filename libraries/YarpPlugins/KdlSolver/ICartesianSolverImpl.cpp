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

yarp::dev::ReturnValue KdlSolver::getNumJoints(std::size_t & numJoints)
{
    numJoints = chain.getNrOfJoints();
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::getNumTcps(std::size_t & numTcps)
{
    numTcps = 1;
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::appendLink(const std::vector<double> & x)
{
    KDL::Frame frameX = KdlVectorConverter::vectorToFrame(x);

    std::lock_guard lock(mtx);

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), frameX));

    fkSolverPos->updateInternalDataStructures();
    ikSolverPos->updateInternalDataStructures();
    ikSolverVel->updateInternalDataStructures();
    idSolver->updateInternalDataStructures();

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::restoreOriginalChain()
{
    std::lock_guard lock(mtx);

    chain = originalChain;

    fkSolverPos->updateInternalDataStructures();
    ikSolverPos->updateInternalDataStructures();
    ikSolverVel->updateInternalDataStructures();
    idSolver->updateInternalDataStructures();

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::changeOrigin(const std::vector<double> & x_old_obj, const std::vector<double> & x_new_old, std::vector<double> & x_new_obj)
{
    KDL::Frame H_old_obj = KdlVectorConverter::vectorToFrame(x_old_obj);
    KDL::Frame H_new_old = KdlVectorConverter::vectorToFrame(x_new_old);
    KDL::Frame H_new_obj = H_new_old * H_old_obj;

    x_new_obj = KdlVectorConverter::frameToVector(H_new_obj);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::forwardKinematics(const std::vector<double> & q, std::vector<double> & x)
{
    KDL::JntArray qInRad(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    KDL::Frame fOutCart;

    {
        std::lock_guard lock(mtx);
        fkSolverPos->JntToCart(qInRad, fOutCart);
    }

    x = KdlVectorConverter::frameToVector(fOutCart);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::poseDiff(const std::vector<double> & xLhs, const std::vector<double> & xRhs, std::vector<double> & xOut)
{
    KDL::Frame fLhs = KdlVectorConverter::vectorToFrame(xLhs);
    KDL::Frame fRhs = KdlVectorConverter::vectorToFrame(xRhs);

    KDL::Twist diff = KDL::diff(fRhs, fLhs); // [fLhs - fRhs] for translation
    xOut = KdlVectorConverter::twistToVector(diff);

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::inverseKinematics(const std::vector<double> & xd, const std::vector<double> & qGuess, std::vector<double> & q, Frame frame)
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
        std::lock_guard lock(mtx);

        if (frame == Frame::TCP)
        {
            KDL::Frame fOutCart;
            fkSolverPos->JntToCart(qGuessInRad, fOutCart);
            frameXd = fOutCart * frameXd;
        }
        else if (frame != Frame::BASE)
        {
            yCWarning(logc, "Unsupported frame");
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }

        ret = ikSolverPos->CartToJnt(qGuessInRad, frameXd, kdlq);
    }

    if (ret < 0)
    {
        yCError(logc, "inverseKinematics(): %s", ikSolverPos->strError(ret));
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
    else if (ret > 0)
    {
        yCWarning(logc, "inverseKinematics(): %s", ikSolverPos->strError(ret));
    }

    q.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        q[motor] = kdlq(motor) * KDL::rad2deg;
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::diffInverseKinematics(const std::vector<double> & q, const std::vector<double> & xdot, std::vector<double> & qdot, Frame frame)
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
        std::lock_guard lock(mtx);

        if (frame == Frame::TCP)
        {
            KDL::Frame fOutCart;
            fkSolverPos->JntToCart(qInRad, fOutCart);

            //-- Transform the basis to which the twist is expressed, but leave the reference point intact
            //-- "Twist and Wrench transformations" @ http://docs.ros.org/indigo/api/orocos_kdl/html/geomprim.html
            kdlxdot = fOutCart.M * kdlxdot;
        }
        else if (frame != Frame::BASE)
        {
            yCWarning(logc, "Unsupported frame");
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
        }

        ret = ikSolverVel->CartToJnt(qInRad, kdlxdot, qDotOutRadS);
    }

    if (ret < 0)
    {
        yCError(logc, "diffInverseKinematics(): %s", ikSolverVel->strError(ret));
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
    else if (ret > 0)
    {
        yCWarning(logc, "diffInverseKinematics(): %s", ikSolverVel->strError(ret));
    }

    qdot.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        qdot[motor] = qDotOutRadS(motor) * KDL::rad2deg;
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::inverseDynamics(const std::vector<double> & q, std::vector<double> & t)
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
        std::lock_guard lock(mtx);
        ret = idSolver->CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt);
    }

    if (ret < 0)
    {
        yCError(logc, "inverseDynamics(): %s", idSolver->strError(ret));
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
    else if (ret > 0)
    {
        yCWarning(logc, "inverseDynamics(): %s", idSolver->strError(ret));
    }

    t.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue KdlSolver::inverseDynamics(const std::vector<double> & q, const std::vector<double> & qdot, const std::vector<double> & qdotdot,
                                                  const std::vector<double> & ftip, std::vector<double> & t, Frame frame)
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
    KDL::Wrench kdlftip = KdlVectorConverter::vectorToWrench(ftip);

    if (frame == Frame::BASE)
    {
        KDL::Frame fOutCart;
        fkSolverPos->JntToCart(qInRad, fOutCart);

        //-- Transform the basis to which the wrench is expressed, but leave the reference point intact
        //-- "Twist and Wrench transformations" @ http://docs.ros.org/indigo/api/orocos_kdl/html/geomprim.html
        kdlftip = fOutCart.M.Inverse() * kdlftip;
    }
    else if (frame != Frame::TCP)
    {
        yCWarning(logc, "Unsupported frame");
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }

    wrenches.back() = kdlftip; // must be expressed in the HN frame

    KDL::JntArray kdlt(chain.getNrOfJoints());
    int ret;

    {
        std::lock_guard lock(mtx);
        ret = idSolver->CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt);
    }

    if (ret < 0)
    {
        yCError(logc, "inverseDynamics(): %s", idSolver->strError(ret));
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
    }
    else if (ret > 0)
    {
        yCWarning(logc, "inverseDynamics(): %s", idSolver->strError(ret));
    }

    t.resize(chain.getNrOfJoints());

    for (int motor = 0; motor < chain.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}

// -----------------------------------------------------------------------------

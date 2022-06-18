// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlTreeSolver.hpp"

#include <yarp/os/LogStream.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/utilities/utility.h> // KDL::deg2rad, KDL::rad2deg

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

int KdlTreeSolver::getNumJoints()
{
    return tree.getNrOfJoints();
}

// -----------------------------------------------------------------------------

int KdlTreeSolver::getNumTcps()
{
    return endpoints.size();
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::appendLink(const std::vector<double> & x)
{
    yCError(KDLS) << "Not supported: appendLink";
    return false;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::restoreOriginalChain()
{
    yCError(KDLS) << "Not supported: restoreOriginalChain";
    return false;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::changeOrigin(const std::vector<double> & x_old_obj, const std::vector<double> & x_new_old, std::vector<double> & x_new_obj)
{
    x_new_obj.clear();
    x_new_obj.reserve(endpoints.size() * 6);

    for (auto i = 0; i < endpoints.size(); i++)
    {
        std::vector<double> temp_old_obj(x_old_obj.cbegin() + i * 6, x_old_obj.cbegin() + (i + 1) * 6);
        std::vector<double> temp_new_old(x_new_old.cbegin() + i * 6, x_new_old.cbegin() + (i + 1) * 6);

        KDL::Frame H_old_obj = KdlVectorConverter::vectorToFrame(temp_old_obj);
        KDL::Frame H_new_old = KdlVectorConverter::vectorToFrame(temp_new_old);
        KDL::Frame H_new_obj = H_new_old * H_old_obj;

        std::vector<double> temp_new_obj = KdlVectorConverter::frameToVector(H_new_obj);
        x_new_obj.insert(x_new_obj.end(), temp_new_obj.cbegin(), temp_new_obj.cend());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::fwdKin(const std::vector<double> & q, std::vector<double> & x)
{
    KDL::JntArray qInRad(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    x.clear();
    x.reserve(endpoints.size() * 6);

    for (const auto & endpoint : endpoints)
    {
        KDL::Frame fOutCart;

        if (fkSolverPos->JntToCart(qInRad, fOutCart, endpoint) < 0)
        {
            return false;
        }

        auto temp = KdlVectorConverter::frameToVector(fOutCart);
        x.insert(x.end(), temp.cbegin(), temp.cend());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::poseDiff(const std::vector<double> & xLhs, const std::vector<double> & xRhs, std::vector<double> & xOut)
{
    xOut.clear();
    xOut.reserve(endpoints.size() * 6);

    for (auto i = 0; i < endpoints.size(); i++)
    {
        std::vector<double> temp_xLhs(xLhs.cbegin() + i * 6, xLhs.cbegin() + (i + 1) * 6);
        std::vector<double> temp_xRhs(xRhs.cbegin() + i * 6, xRhs.cbegin() + (i + 1) * 6);

        KDL::Frame fLhs = KdlVectorConverter::vectorToFrame(temp_xLhs);
        KDL::Frame fRhs = KdlVectorConverter::vectorToFrame(temp_xRhs);

        KDL::Twist diff = KDL::diff(fRhs, fLhs); // [fLhs - fRhs] for translation
        std::vector<double> temp_xOut = KdlVectorConverter::twistToVector(diff);

        xOut.insert(xOut.end(), temp_xOut.cbegin(), temp_xOut.cend());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::invKin(const std::vector<double> & xd, const std::vector<double> & qGuess, std::vector<double> & q, const reference_frame frame)
{
    KDL::Frames frames;
    int i = 0;

    for (const auto & endpoint : endpoints)
    {
        if (!mergedEndpoints.empty() && mergedEndpoints.find(endpoint) != mergedEndpoints.end())
        {
            frames.emplace(endpoint, frames[mergedEndpoints[endpoint]]);
        }
        else
        {
            std::vector<double> sub(xd.cbegin() + i * 6, xd.cbegin() + (i + 1) * 6);
            frames.emplace(endpoint, KdlVectorConverter::vectorToFrame(sub));
            i++;
        }
    }

    KDL::JntArray qGuessInRad(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        qGuessInRad(motor) = qGuess[motor] * KDL::deg2rad;
    }

    if (frame == TCP_FRAME)
    {
        for (const auto & endpoint : endpoints)
        {
            KDL::Frame fOutCart;

            if (fkSolverPos->JntToCart(qGuessInRad, fOutCart, endpoint) < 0)
            {
                return false;
            }

            auto it = frames.find(endpoint);
            it->second = fOutCart * it->second;
        }
    }
    else if (frame != BASE_FRAME)
    {
        yCWarning(KDLS) << "Unsupported frame";
        return false;
    }

    KDL::JntArray kdlq(tree.getNrOfJoints());

    if (ikSolverPos->CartToJnt(qGuessInRad, frames, kdlq) < 0)
    {
        return false;
    }

    q.resize(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        q[motor] = kdlq(motor) * KDL::rad2deg;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::diffInvKin(const std::vector<double> & q, const std::vector<double> & xdot, std::vector<double> & qdot, const reference_frame frame)
{
    KDL::Twists twists;
    int i = 0;

    for (const auto & endpoint : endpoints)
    {
        if (!mergedEndpoints.empty() && mergedEndpoints.find(endpoint) != mergedEndpoints.end())
        {
            twists.emplace(endpoint, twists[mergedEndpoints[endpoint]]);
        }
        else
        {
            std::vector<double> sub(xdot.cbegin() + i * 6, xdot.cbegin() + (i + 1) * 6);
            twists.emplace(endpoint, KdlVectorConverter::vectorToTwist(sub));
            i++;
        }
    }

    KDL::JntArray qInRad(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    if (frame == TCP_FRAME)
    {
        for (const auto & endpoint : endpoints)
        {
            KDL::Frame fOutCart;

            if (fkSolverPos->JntToCart(qInRad, fOutCart, endpoint) < 0)
            {
                return false;
            }

            auto it = twists.find(endpoint);

            //-- Transform the basis to which the twist is expressed, but leave the reference point intact
            //-- "Twist and Wrench transformations" @ http://docs.ros.org/latest/api/orocos_kdl/html/geomprim.html
            it->second = fOutCart.M * it->second;
        }
    }
    else if (frame != BASE_FRAME)
    {
        yCWarning(KDLS) << "Unsupported frame";
        return false;
    }

    KDL::JntArray qDotOutRadS(tree.getNrOfJoints());

    if (ikSolverVel->CartToJnt(qInRad, twists, qDotOutRadS) < 0)
    {
        return false;
    }

    qdot.resize(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        qdot[motor] = qDotOutRadS(motor) * KDL::rad2deg;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::invDyn(const std::vector<double> & q, std::vector<double> & t)
{
    KDL::JntArray qInRad(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    KDL::JntArray qdotInRad(tree.getNrOfJoints());
    KDL::JntArray qdotdotInRad(tree.getNrOfJoints());
    KDL::JntArray kdlt(tree.getNrOfJoints());
    KDL::WrenchMap wrenches;

    for (const auto & endpoint : endpoints)
    {
        wrenches.emplace(endpoint, KDL::Wrench::Zero());
    }

    if (idSolver->CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt) < 0)
    {
        return false;
    }

    t.resize(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::invDyn(const std::vector<double> & q, const std::vector<double> & qdot, const std::vector<double> & qdotdot, const std::vector<double> & ftip, std::vector<double> & t)
{
    KDL::JntArray qInRad(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        qInRad(motor) = q[motor] * KDL::deg2rad;
    }

    KDL::JntArray qdotInRad(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        qdotInRad(motor) = qdot[motor] * KDL::deg2rad;
    }

    KDL::JntArray qdotdotInRad(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        qdotdotInRad(motor) = qdotdot[motor] * KDL::deg2rad;
    }

    KDL::WrenchMap wrenches;

    for (const auto & endpoint : endpoints)
    {
        // FIXME: not trivial, see https://github.com/roboticslab-uc3m/kinematics-dynamics/issues/162
        return false;
    }

    KDL::JntArray kdlt(tree.getNrOfJoints());

    if (idSolver->CartToJnt(qInRad, qdotInRad, qdotdotInRad, wrenches, kdlt) < 0)
    {
        return false;
    }

    t.resize(tree.getNrOfJoints());

    for (int motor = 0; motor < tree.getNrOfJoints(); motor++)
    {
        t[motor] = kdlt(motor);
    }

    return true;
}

// -----------------------------------------------------------------------------

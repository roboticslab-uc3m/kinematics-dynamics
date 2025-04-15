// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

#include <cmath> // std::pow

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include <kdl/jntarray.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include "ConfigurationSelector.hpp"

#include "ChainIkSolverPos_ST.hpp"
#include "ChainIkSolverPos_ID.hpp"

#include "KdlSolverUtils.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool KdlSolver::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(KDLS) << "Failed to parse parameters";
        return false;
    }

    yarp::os::Property kinematicsConfig;

    if (!KdlSolverUtils::parseKinematicsConfig(config, m_kinematics, kinematicsConfig))
    {
        yCError(KDLS) << "Failed to parse kinematics configuration";
        return false;
    }

    if (!KdlSolverUtils::makeChain(kinematicsConfig, chain))
    {
        yCError(KDLS) << "Failed to create kinematic chain";
        return false;
    }

    if (m_gravity.size() != 3)
    {
        yCError(KDLS) << "Gravity vector must have exactly 3 elements";
        return false;
    }

    if (m_mins.size() != m_maxs.size() || m_mins.size() != chain.getNrOfJoints())
    {
        yCError(KDLS) << "Vectors of joints limits must have exactly" << chain.getNrOfJoints() << "elements";
        return false;
    }

    KDL::Vector gravity(m_gravity[0], m_gravity[1], m_gravity[2]);

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    idSolver = new KDL::ChainIdSolver_RNE(chain, gravity);

    if (m_ikVel == "pinv")
    {
        ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, m_epsVel, m_maxIterVel);
    }
    else if (m_ikVel == "wdls")
    {
        ikSolverVel = new KDL::ChainIkSolverVel_wdls(chain, m_epsVel, m_maxIterVel);

        auto * temp = dynamic_cast<KDL::ChainIkSolverVel_wdls *>(ikSolverVel);
        temp->setLambda(m_lambda);

        if (m_weightsJS.size() != std::pow(chain.getNrOfJoints(), 2))
        {
            yCWarning(KDLS) << "Illegal size of weightJS";
        }

        temp->setWeightJS(KdlSolverUtils::getMatrixFromVector(m_weightsJS));

        if (m_weightsTS.size() != std::pow(6, 2))
        {
            yCWarning(KDLS) << "Illegal size of weightTS";
        }

        temp->setWeightTS(KdlSolverUtils::getMatrixFromVector(m_weightsTS));
    }
    else
    {
        yCError(KDLS) << "Unsupported IK velocity solver algorithm:" << m_ikVel;
        return false;
    }

    if (m_ikPos == "lma")
    {
        Eigen::Matrix<double, 6, 1> L(m_weightsLMA.data());
        ikSolverPos = new KDL::ChainIkSolverPos_LMA(chain, L, m_epsPos, m_maxIterPos);
    }
    else if (m_ikPos == "nrjl")
    {
        KDL::JntArray qMax = KdlSolverUtils::getJntArrayFromVector(m_maxs);
        KDL::JntArray qMin = KdlSolverUtils::getJntArrayFromVector(m_mins);

        ikSolverPos = new KDL::ChainIkSolverPos_NR_JL(chain, qMin, qMax, *fkSolverPos, *ikSolverVel, m_maxIterPos, m_epsPos);
    }
    else if (m_ikPos == "st")
    {
        KDL::JntArray qMax = KdlSolverUtils::getJntArrayFromVector(m_maxs);
        KDL::JntArray qMin = KdlSolverUtils::getJntArrayFromVector(m_mins);

        if (m_invKinStrategy == "leastOverallAngularDisplacement")
        {
            ConfigurationSelectorLeastOverallAngularDisplacementFactory factory(qMin, qMax);
            ikSolverPos = ChainIkSolverPos_ST::create(chain, factory);
        }
        else if (m_invKinStrategy == "humanoidGait")
        {
            ConfigurationSelectorHumanoidGaitFactory factory(qMin, qMax);
            ikSolverPos = ChainIkSolverPos_ST::create(chain, factory);
        }
        else
        {
            yCError(KDLS) << "Unsupported IK strategy:" << m_invKinStrategy;
            return false;
        }

        if (!ikSolverPos)
        {
            yCError(KDLS) << "Unable to solve IK";
            return false;
        }
    }
    else if (m_ikPos == "id")
    {
        KDL::JntArray qMax = KdlSolverUtils::getJntArrayFromVector(m_maxs);
        KDL::JntArray qMin = KdlSolverUtils::getJntArrayFromVector(m_mins);

        ikSolverPos = new ChainIkSolverPos_ID(chain, qMin, qMax, *fkSolverPos);
    }
    else
    {
        yCError(KDLS) << "Unsupported IK position solver algorithm:" << m_ikPos;
        return false;
    }

    originalChain = chain;

    return true;
}

// -----------------------------------------------------------------------------

bool KdlSolver::close()
{
    delete fkSolverPos;
    fkSolverPos = nullptr;

    delete ikSolverPos;
    ikSolverPos = nullptr;

    delete ikSolverVel;
    ikSolverVel = nullptr;

    delete idSolver;
    idSolver = nullptr;

    return true;
}

// -----------------------------------------------------------------------------

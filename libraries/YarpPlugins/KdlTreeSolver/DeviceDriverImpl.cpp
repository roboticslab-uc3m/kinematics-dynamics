// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlTreeSolver.hpp"

#include <cmath> // std::pow

#include <algorithm> // std::find
#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/utilities/utility.h> // KDL::deg2rad

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treeiksolverpos_online.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>

#include "KdlSolverUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool KdlTreeSolver::open(yarp::os::Searchable & config)
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

    if (!makeTree(kinematicsConfig))
    {
        yCError(KDLS) << "Failed to create kinematic tree";
        return false;
    }

    if (m_gravity.size() != 3)
    {
        yCError(KDLS) << "Gravity vector must have exactly 3 elements";
        return false;
    }

    if (m_mins.size() != m_maxs.size() || m_mins.size() != m_maxvels.size() || m_mins.size() != tree.getNrOfJoints())
    {
        yCError(KDLS) << "Vectors of joints limits must have exactly" << tree.getNrOfJoints() << "elements";
        return false;
    }

    KDL::Vector gravity(m_gravity[0], m_gravity[1], m_gravity[2]);

    fkSolverPos = new KDL::TreeFkSolverPos_recursive(tree);
    ikSolverVel = new KDL::TreeIkSolverVel_wdls(tree, endpoints);
    idSolver = new KDL::TreeIdSolver_RNE(tree, gravity);

    {
        auto * temp = dynamic_cast<KDL::TreeIkSolverVel_wdls *>(ikSolverVel);
        temp->setLambda(m_lambda);

        if (m_weightsJS.size() != std::pow(tree.getNrOfJoints(), 2))
        {
            yCWarning(KDLS) << "Illegal size of weightJS";
        }

        temp->setWeightJS(KdlSolverUtils::getMatrixFromVector(m_weightsJS));

        if (m_weightsTS.size() != std::pow(endpoints.size() * 6, 2))
        {
            yCWarning(KDLS) << "Illegal size of weightTS";
        }

        temp->setWeightTS(KdlSolverUtils::getMatrixFromVector(m_weightsTS));
    }

    KDL::JntArray qMax = KdlSolverUtils::getJntArrayFromVector(m_maxs);
    KDL::JntArray qMin = KdlSolverUtils::getJntArrayFromVector(m_mins);

    if (m_ikPos == "nrjl")
    {
        ikSolverPos = new KDL::TreeIkSolverPos_NR_JL(tree, endpoints, qMin, qMax, *fkSolverPos, *ikSolverVel, m_maxIterPos, m_epsPos);
    }
    else if (m_ikPos == "online")
    {
        KDL::JntArray qMaxVels = KdlSolverUtils::getJntArrayFromVector(m_maxvels);
        ikSolverPos = new KDL::TreeIkSolverPos_Online(tree.getNrOfJoints(), endpoints, qMin, qMax, qMaxVels, m_vTranslMax, m_vRotMax * KDL::deg2rad, *fkSolverPos, *ikSolverVel);
    }
    else
    {
        yCError(KDLS) << "Unsupported IK solver algorithm:" << m_ikPos;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::makeTree(const yarp::os::Searchable & kinematicsConfig)
{
    yarp::os::Bottle chains = kinematicsConfig.findGroup("chain", "kinematic chains").tail();

    if (chains.isNull() || chains.size() == 0)
    {
        yCError(KDLS) << "Missing or empty \"chain\" section collection";
        return false;
    }

    for (int i = 0; i < chains.size(); i++)
    {
        KDL::Chain chain;

        auto chainName = chains.get(i).asString();
        yCInfo(KDLS) << "chain:" << chainName;

        const yarp::os::Bottle & chainConfig = kinematicsConfig.findGroup(chainName);
        yCDebug(KDLS) << "config:" << chainConfig.toString();

        if (!KdlSolverUtils::makeChain(kinematicsConfig, chain))
        {
            yCError(KDLS) << "Failed to create kinematic chain";
            return false;
        }

        auto hook = chainConfig.check("hook", yarp::os::Value("root"), "segment to hook to at its end").asString();
        yCInfo(KDLS) << "hook:" << hook;

        if (!tree.addChain(chain, hook))
        {
            yCError(KDLS) << "Unable to add chain to tree, hook name" << hook << "not found";
            return false;
        }

        if (chainConfig.check("endpoint", yarp::os::Value(false), "whether this chain is an endpoint (TCP)").asBool())
        {
            endpoints.push_back(chainName);

            if (chainConfig.check("mergeWith", "other chain's TCP to merge this TCP with"))
            {
                auto mergeWith = chainConfig.find("mergeWith").asString();

                if (std::find(endpoints.cbegin(), endpoints.cend(), mergeWith) == endpoints.cend())
                {
                    yCError(KDLS) << "Unable to find TCP" << mergeWith << "to be merged with" << chainName;
                    return false;
                }

                mergedEndpoints.emplace(chainName, mergeWith);
                yCInfo(KDLS) << "TCP" << chainName << "merged with" << mergeWith;
            }
        }
    }

    yCInfo(KDLS) << "Tree number of segments:" << tree.getNrOfSegments();
    yCInfo(KDLS) << "Tree number of joints:" << tree.getNrOfJoints();
    yCInfo(KDLS) << "Tree endpoints:" << endpoints;

    return true;
}

// -----------------------------------------------------------------------------

bool KdlTreeSolver::close()
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

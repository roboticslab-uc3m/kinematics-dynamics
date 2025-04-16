// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

#include <yarp/os/LogStream.h>

#include <yarp/math/Math.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto NUM_MOTORS = 5;

// ------------------- DeviceDriver Related ------------------------------------

bool AsibotSolver::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(ASIBOT) << "Failed to parse parameters";
        return false;
    }

    if (m_mins.size() != NUM_MOTORS || m_maxs.size() != NUM_MOTORS)
    {
        yCError(ASIBOT) << "Vectors of joints limits must have exactly" << NUM_MOTORS << "elements";
        return false;
    }

    if (m_invKinStrategy == m_invKinStrategy_defaultValue)
    {
        confFactory = new AsibotConfigurationLeastOverallAngularDisplacementFactory(m_mins, m_maxs);
    }
    else
    {
        yCError(ASIBOT) << "Unsupported IK configuration strategy:" << m_invKinStrategy;
        return false;
    }

    tcpFrameStruct.hasFrame = false;
    tcpFrameStruct.frameTcp = yarp::math::eye(4);

    return true;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::close()
{
    if (confFactory)
    {
        delete confFactory;
        confFactory = nullptr;
    }

    return true;
}

// -----------------------------------------------------------------------------

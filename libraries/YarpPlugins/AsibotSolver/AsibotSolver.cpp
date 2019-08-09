// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::buildStrategyFactory(const std::string & strategy)
{
    if (strategy == DEFAULT_STRATEGY)
    {
        confFactory = new AsibotConfigurationLeastOverallAngularDisplacementFactory(qMin, qMax);
    }
    else
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

roboticslab::AsibotConfiguration * roboticslab::AsibotSolver::getConfiguration() const
{
    return confFactory->create();
}

// -----------------------------------------------------------------------------

roboticslab::AsibotSolver::AsibotTcpFrame roboticslab::AsibotSolver::getTcpFrame() const
{
    std::lock_guard<std::mutex> lock(mtx);
    return tcpFrameStruct;
}

// -----------------------------------------------------------------------------

void roboticslab::AsibotSolver::setTcpFrame(const AsibotTcpFrame & tcpFrameStruct)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->tcpFrameStruct = tcpFrameStruct;
}

// -----------------------------------------------------------------------------

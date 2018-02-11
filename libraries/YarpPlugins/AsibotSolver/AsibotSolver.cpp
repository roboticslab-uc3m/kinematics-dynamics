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
    AsibotTcpFrame tcpFrameStructLocal;
    mutex.wait();
    tcpFrameStructLocal = tcpFrameStruct;
    mutex.post();
    return tcpFrameStructLocal;
}

// -----------------------------------------------------------------------------

void roboticslab::AsibotSolver::setTcpFrame(const AsibotTcpFrame & tcpFrameStruct)
{
    mutex.wait();
    this->tcpFrameStruct = tcpFrameStruct;
    mutex.post();
}

// -----------------------------------------------------------------------------

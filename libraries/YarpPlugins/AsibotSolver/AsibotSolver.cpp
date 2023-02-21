// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

AsibotConfiguration * AsibotSolver::getConfiguration() const
{
    return confFactory->create();
}

// -----------------------------------------------------------------------------

AsibotSolver::AsibotTcpFrame AsibotSolver::getTcpFrame() const
{
    std::lock_guard lock(mtx);
    return tcpFrameStruct;
}

// -----------------------------------------------------------------------------

void AsibotSolver::setTcpFrame(const AsibotTcpFrame & tcpFrameStruct)
{
    std::lock_guard lock(mtx);
    this->tcpFrameStruct = tcpFrameStruct;
}

// -----------------------------------------------------------------------------

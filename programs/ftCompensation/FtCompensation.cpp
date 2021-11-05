// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FtCompensation.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool FtCompensation::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(FTC) << "Config:" << rf.toString();
    return true;
}

bool FtCompensation::updateModule()
{
    return true;
}

bool FtCompensation::interruptModule()
{
    return true;
}

double FtCompensation::getPeriod()
{
    return 0.01; // [s]
}

bool FtCompensation::close()
{
    return true;
}

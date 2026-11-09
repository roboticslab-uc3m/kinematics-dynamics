#include "HapticFeedback.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool HapticFeedback::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(HF) << "Config:" << rf.toString();
    return true;
}

bool HapticFeedback::updateModule()
{
    return true;
}

bool HapticFeedback::interruptModule()
{
    return true;
}

bool HapticFeedback::close()
{
    return true;
}

double HapticFeedback::getPeriod()
{
    return 0.01; // 10 ms period
}

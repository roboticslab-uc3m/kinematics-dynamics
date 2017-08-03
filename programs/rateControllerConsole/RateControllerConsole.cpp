#include "RateControllerConsole.hpp"

#include <string>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <ColorDebug.hpp>

namespace roboticslab
{

bool RateControllerConsole::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("StreamingSpnav config: %s.\n", rf.toString().c_str());
    return true;
}

bool RateControllerConsole::updateModule()
{
    return true;
}

bool RateControllerConsole::interruptModule()
{
    return true;
}

double RateControllerConsole::getPeriod()
{
    return 0.02;  // [s]
}

}  // namespace roboticslab

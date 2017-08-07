#include <cstdio>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <ColorDebug.hpp>

#include "RateControllerConsole.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup rateControllerConsole rateControllerConsole
 *
 * @brief Creates an instance of roboticslab::RateControllerConsole.
 */

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("rateControllerConsole");
    rf.setDefaultConfigFile("rateControllerConsole.ini");
    rf.configure(argc, argv);

    roboticslab::RateControllerConsole mod;

    CD_INFO("rateControllerConsole checking for yarp network... ");
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("rateControllerConsole found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");

    return mod.runModule(rf);
}

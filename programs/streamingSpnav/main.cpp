#include <cstdio>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <ColorDebug.hpp>

#include "StreamingSpnav.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup streamingSpnav streamingSpnav
 *
 * @brief Creates an instance of roboticslab::StreamingSpnav.
 */

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("streamingSpnav");
    rf.setDefaultConfigFile("streamingSpnav.ini");
    rf.configure(argc, argv);

    roboticslab::StreamingSpnav mod;

    CD_INFO("streamingSpnav checking for yarp network... ");
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("streamingSpnav found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");

    return mod.runModule(rf);
}

#include <cstdio>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <ColorDebug.h>

#include "StreamingDeviceController.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup streamingDeviceController streamingDeviceController
 *
 * @brief Creates an instance of roboticslab::StreamingDeviceController.
 */

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("streamingDeviceController");
    rf.setDefaultConfigFile("streamingDeviceController.ini");
    rf.configure(argc, argv);

    roboticslab::StreamingDeviceController mod;

    CD_INFO("streamingDeviceController checking for yarp network... ");
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("streamingDeviceController found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");

    if (mod.configure(rf))
    {
        return mod.runModule();
    }
    else
    {
        return mod.close() ? 0 : 1;
    }
}

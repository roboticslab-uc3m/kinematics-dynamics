#include <cstdio>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <ColorDebug.h>

#include "KeyboardController.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup keyboardController keyboardController
 *
 * @brief Creates an instance of roboticslab::KeyboardController.
 */

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("keyboardController");
    rf.setDefaultConfigFile("keyboardController.ini");
    rf.configure(argc, argv);

    roboticslab::KeyboardController mod;

    CD_INFO("keyboardController checking for yarp network... ");
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("keyboardController found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");

    return mod.runModule(rf);
}

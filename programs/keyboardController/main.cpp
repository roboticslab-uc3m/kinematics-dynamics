// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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

    if (mod.configure(rf))
    {
        return mod.runModule();
    }
    else
    {
        return mod.close() ? 0 : 1;
    }
}

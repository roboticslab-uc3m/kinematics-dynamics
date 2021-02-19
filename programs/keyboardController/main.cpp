// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstdio>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

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

    yInfo() << "keyboardController checking for yarp network...";
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "keyboardController found no yarp network (try running \"yarpserver &\"), bye!";
        return 1;
    }

    if (mod.configure(rf))
    {
        return mod.runModule();
    }
    else
    {
        return mod.close() ? 0 : 1;
    }
}

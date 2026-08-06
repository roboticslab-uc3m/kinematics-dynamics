// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstdio>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include "VisualServoController.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * @defgroup visualServoController visualServoController
 *
 * @brief Creates an instance of roboticslab::VisualServoController.
 */

int main(int argc, char** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("visualServoController");
    rf.setDefaultConfigFile("visualServoController.ini");
    rf.configure(argc, argv);

    roboticslab::VisualServoController mod;

    yInfo() << "visualServoController checking for yarp network...";
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "visualServoController found no yarp network (try running \"yarpserver &\"), bye!";
        return 1;
    }

    return mod.runModule(rf);
}

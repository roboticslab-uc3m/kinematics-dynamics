// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstdio>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <ColorDebug.h>

#include "HaarDetectionController.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * @defgroup haarDetectionController haarDetectionController
 *
 * @brief Creates an instance of roboticslab::HaarDetectionController.
 */

int main(int argc, char** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("haarDetectionController");
    rf.setDefaultConfigFile("haarDetectionController.ini");
    rf.configure(argc, argv);

    roboticslab::HaarDetectionController mod;

    CD_INFO("haarDetectionController checking for yarp network... ");
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("haarDetectionController found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");

    return mod.runModule(rf);
}

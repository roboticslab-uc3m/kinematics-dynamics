// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include "FtCompensation.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * @defgroup ftCompensation ftCompensation
 *
 * @brief Creates an instance of roboticslab::FtCompensation.
 */

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("ftCompensation");
    rf.setDefaultConfigFile("ftCompensation.ini");
    rf.configure(argc, argv);

    roboticslab::FtCompensation mod;

    yInfo() << "ftCompensation checking for YARP network...";

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "ftCompensation found no YARP network (try running \"yarpserver &\")";
        return 1;
    }

    return mod.runModule(rf);
}

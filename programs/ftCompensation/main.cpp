// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstdio>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include "FtCompensation.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup ftCompensation ftCompensation
 *
 * @brief Creates an instance of roboticslab::FtCompensation.
 */

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("ftCompensation");
    rf.setDefaultConfigFile("ftCompensation.ini");
    rf.configure(argc, argv);

    roboticslab::FtCompensation mod;

    yInfo() << "ftCompensation checking for yarp network...";
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "ftCompensation found no yarp network (try running \"yarpserver &\"), bye!";
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

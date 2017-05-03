// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup vision_programs
 *
 * \defgroup transCoordsUsingJoints transCoordsUsingJoints
 *
 * @brief Creates an instance of teo::TransCoordsUsingJoints.
 */

#include "TransCoordsUsingJoints.hpp"

int main(int argc, char *argv[]) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("transCoordsUsingJoints");
    rf.setDefaultConfigFile("transCoordsUsingJoints.ini");
    rf.configure(argc, argv);

    teo::TransCoordsUsingJoints mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"transCoordsUsingJoints --help\" for options.\n");
    printf("premultH checking for yarp network... ");
    fflush(stdout);
    yarp::os::Network yarp;
    if ( ! yarp.checkNetwork() )
    {
        fprintf(stderr, "[fail]\ntransCoordsUsingJoints found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}


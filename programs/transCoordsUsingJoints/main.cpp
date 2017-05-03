// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup vision_programs
 *
 * \defgroup cv1ToRoot cv1ToRoot
 *
 * @brief Creates an instance of teo::Cv1ToRoot.
 */

#include "Cv1ToRoot.hpp"

int main(int argc, char *argv[]) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cv1ToRoot");
    rf.setDefaultConfigFile("cv1ToRoot.ini");
    rf.configure(argc, argv);

    teo::Cv1ToRoot mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"cv1ToRoot --help\" for options.\n");
    printf("premultH checking for yarp network... ");
    fflush(stdout);
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stderr, "[fail]\ncv1ToRoot found no yarp network (try running \"yarpserver &\"), bye!\n");
        return -1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}


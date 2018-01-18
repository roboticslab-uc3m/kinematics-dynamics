#include <cstdio>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <ColorDebug.hpp>

#include "TransCoordsUsingJoints.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup transCoordsUsingJoints transCoordsUsingJoints
 *
 * @brief Creates an instance of roboticslab::TransCoordsUsingJoints.
 *
 * Use example: transCoordsUsingJoints --kinematics /usr/local/share/teo-configuration-files/contexts/kinematics/headKinematics.ini --local /transCoordsUsingJoints --remote /teo/head
 */

int main(int argc, char *argv[]) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("transCoordsUsingJoints");
    rf.setDefaultConfigFile("transCoordsUsingJoints.ini");
    rf.configure(argc, argv);

    roboticslab::TransCoordsUsingJoints mod;
    if(rf.check("help")) {
        CD_INFO("Use example: transCoordsUsingJoints --kinematics /usr/local/share/teo-configuration-files/contexts/kinematics/headKinematics.ini --local /transCoordsUsingJoints --remote /teo/head\n");
        return mod.runModule(rf);
    }

    std::printf("Run \"transCoordsUsingJoints --help\" for options.\n");
    std::printf("premultH checking for yarp network... ");
    std::fflush(stdout);
    yarp::os::Network yarp;
    if ( ! yarp.checkNetwork() )
    {
        std::fprintf(stderr, "[fail]\ntransCoordsUsingJoints found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else std::printf("[ok]\n");

    return mod.runModule(rf);
}

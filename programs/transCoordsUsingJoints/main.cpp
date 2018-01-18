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

    CD_INFO_NO_HEADER("Run \"transCoordsUsingJoints --help\" for options.\n");
    CD_INFO_NO_HEADER("premultH checking for yarp network... ");
    yarp::os::Network yarp;
    if ( ! yarp.checkNetwork() )
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("transCoordsUsingJoints found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");

    return mod.runModule(rf);
}

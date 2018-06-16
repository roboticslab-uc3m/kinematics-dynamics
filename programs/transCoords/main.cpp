#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <ColorDebug.h>

#include "TransCoords.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup transCoords transCoords
 *
 * @brief Creates an instance of roboticslab::TransCoords.
 *
 * Use example: transCoords --kinematics /usr/local/share/teo-configuration-files/contexts/kinematics/headKinematics.ini --local /transCoords --remote /teo/head
 */

int main(int argc, char *argv[]) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("transCoords");
    rf.setDefaultConfigFile("transCoords.ini");
    rf.configure(argc, argv);

    roboticslab::TransCoords mod;
    if(rf.check("help")) {
        CD_INFO("Use example: transCoords --kinematics /usr/local/share/teo-configuration-files/contexts/kinematics/headKinematics.ini --local /transCoords --remote /teo/head\n");
        return mod.runModule(rf);
    }

    CD_INFO_NO_HEADER("Run \"transCoords --help\" for options.\n");
    CD_INFO_NO_HEADER("premultH checking for yarp network... ");
    yarp::os::Network yarp;
    if ( ! yarp.checkNetwork() )
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("transCoords found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");

    return mod.runModule(rf);
}

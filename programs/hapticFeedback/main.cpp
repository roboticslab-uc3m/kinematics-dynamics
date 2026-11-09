#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include "HapticFeedback.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup hapticFeedback hapticFeedback
 *
 * @brief Creates an instance of roboticslab::HapticFeedback.
 */

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("hapticFeedback");
    rf.setDefaultConfigFile("hapticFeedback.ini");
    rf.configure(argc, argv);

    roboticslab::HapticFeedback mod;

    yInfo() << "hapticFeedback checking for yarp network...";

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "hapticFeedback found no yarp network (try running \"yarpserver &\"), bye!";
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

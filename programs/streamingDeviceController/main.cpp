#include <cstdio>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include "StreamingDeviceController.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup streamingDeviceController streamingDeviceController
 *
 * @brief Creates an instance of roboticslab::StreamingDeviceController.
 */

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("streamingDeviceController");
    rf.setDefaultConfigFile("streamingDeviceController.ini");
    rf.configure(argc, argv);

    roboticslab::StreamingDeviceController mod;

    yInfo() << "streamingDeviceController checking for yarp network...";
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "streamingDeviceController found no yarp network (try running \"yarpserver &\"), bye!";
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

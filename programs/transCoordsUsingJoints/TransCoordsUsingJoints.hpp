#ifndef __TRANS_COORDS_USING_JOINTS_HPP__
#define __TRANS_COORDS_USING_JOINTS_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include "PremultPorts.hpp"

#define DEFAULT_WATCHDOG    5       // [s]


namespace teo
{

/**
 * @ingroup transCoordsUsingJoints
 *
 * @brief Transform Computer Vision values to root frame.
 */
class TransCoordsUsingJoints : public yarp::os::RFModule {
    protected:
        bool updateModule();
        bool interruptModule();
        double getPeriod();
        double watchdog; // [s]

        yarp::os::Port outPort;
        PremultPorts premultPorts;

    public:
        bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace teo

#endif  // __TRANS_COORDS_USING_JOINTS_HPP__


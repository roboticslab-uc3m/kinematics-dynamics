#ifndef __TRANS_COORDS_USING_JOINTS_HPP__
#define __TRANS_COORDS_USING_JOINTS_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include "PremultPorts.hpp"

#include "ColorDebug.hpp"

#define DEFAULT_WATCHDOG    5       // [s]


namespace teo
{

/**
 * @ingroup transCoordsUsingJoints
 *
 * @brief Transform Computer Vision values to root frame.
 */
class TransCoordsUsingJoints : public yarp::os::RFModule
{

public:
    bool configure(yarp::os::ResourceFinder &rf);

private:
    bool updateModule();
    bool interruptModule();
    double getPeriod();

    yarp::os::Port outPort;
    PremultPorts premultPorts;

    yarp::dev::PolyDriver robotDevice;

};

}  // namespace teo

#endif  // __TRANS_COORDS_USING_JOINTS_HPP__


#ifndef __TRANS_COORDS_USING_JOINTS_HPP__
#define __TRANS_COORDS_USING_JOINTS_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Searchable.h>

#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include "PremultPorts.hpp"

#define DEFAULT_SOLVER "KdlSolver"
#define DEFAULT_ROBOT "remote_controlboard"

namespace roboticslab
{

/**
 * @ingroup transCoords
 *
 * @brief Transform values to root frame.
 */
class TransCoords : public yarp::os::RFModule
{

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();
    bool interruptModule();
    double getPeriod();

private:
    bool getMatrixFromProperties(const yarp::os::Bottle &b, KDL::Frame &frame);

    yarp::os::Port outPort;
    PremultPorts premultPorts;

    yarp::dev::PolyDriver robotDevice;
    yarp::dev::PolyDriver solverDevice;

    KDL::Frame H0;

    bool useRobot;
};

}  // namespace roboticslab

#endif  // __TRANS_COORDS_USING_JOINTS_HPP__

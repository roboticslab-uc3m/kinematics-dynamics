#ifndef __TRANS_COORDS_HPP__
#define __TRANS_COORDS_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <kdl/frames.hpp>

#include "KinematicRepresentation.hpp"
#include "ICartesianSolver.h"

#define DEFAULT_SOLVER "KdlSolver"
#define DEFAULT_ROBOT "remote_controlboard"
#define DEFAULT_ANGLE_REPR "axisAngleScaled"

namespace roboticslab
{

/**
 * @ingroup transCoords
 *
 * @brief Transform values to root frame.
 */
class TransCoords : public yarp::os::RFModule,
                    public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual double getPeriod();
    virtual void onRead(yarp::os::Bottle &b);

private:
    bool getMatrixFromProperties(const yarp::os::Bottle &b, KDL::Frame &frame);

    yarp::os::BufferedPort<yarp::os::Bottle> inPort;
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;

    yarp::dev::PolyDriver robotDevice;
    yarp::dev::PolyDriver solverDevice;

    yarp::dev::IEncoders* iEncoders;
    yarp::dev::IControlLimits* iControlLimits;
    roboticslab::ICartesianSolver* iCartesianSolver;

    KDL::Frame fixedH;

    int numRobotJoints;
    bool useRobot;

    KinRepresentation::orientation_system orient;
};

}  // namespace roboticslab

#endif  // __TRANS_COORDS_HPP__

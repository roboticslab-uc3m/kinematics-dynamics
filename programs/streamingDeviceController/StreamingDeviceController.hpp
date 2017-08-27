#ifndef __STREAMING_DEVICE_CONTROLLER_HPP__
#define __STREAMING_DEVICE_CONTROLLER_HPP__

#include <vector>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include "ICartesianControl.h"

#define DEFAULT_DEVICE_PORT_LOCAL "/StreamingDeviceClient"
#define DEFAULT_DEVICE_PORT_REMOTE "/spacenavigator/mouse"

#define DEFAULT_CARTESIAN_LOCAL "/StreamingDeviceCartesianControlClient"
#define DEFAULT_CARTESIAN_REMOTE "/asibotSim/BasicCartesianControl"

#define DEFAULT_SCALING 10.0

#define DEFAULT_FIXED_AXES "none"

namespace roboticslab
{

/**
 * @ingroup streamingDeviceController
 *
 * @brief Sends streaming commands to the cartesian controller from
 * a streaming input device like the 3Dconnexion Space Navigator.
 */
class StreamingDeviceController : public yarp::os::RFModule
{

public:
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual double getPeriod();

private:
    yarp::dev::PolyDriver streamingClientDevice;
    yarp::dev::PolyDriver cartesianControlClientDevice;

    yarp::dev::IAnalogSensor *iAnalogSensor;
    roboticslab::ICartesianControl *iCartesianControl;

    double scaling;

    std::vector<bool> fixedAxes;  // 'true': disabled (fixed axis), 'false': enabled

    bool isStopped;
};

}  // namespace roboticslab

#endif  // __STREAMING_DEVICE_CONTROLLER_HPP__

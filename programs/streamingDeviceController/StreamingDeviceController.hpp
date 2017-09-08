#ifndef __STREAMING_DEVICE_CONTROLLER_HPP__
#define __STREAMING_DEVICE_CONTROLLER_HPP__

#include <vector>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include "StreamingDevice.hpp"

#include "ICartesianControl.h"
#include "IProximitySensors.h"

#define DEFAULT_DEVICE_NAME "SpaceNavigator"

#define DEFAULT_CARTESIAN_LOCAL "/StreamingDeviceCartesianControlClient"
#define DEFAULT_CARTESIAN_REMOTE "/CartesianControl"

#define DEFAULT_PROXIMITY_SENSORS "/sensor_reader"

#define DEFAULT_PERIOD 0.02  // [s]
#define DEFAULT_SCALING 10.0

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
    StreamingDevice * streamingDevice;

    yarp::dev::PolyDriver cartesianControlClientDevice;
    yarp::dev::PolyDriver sensorsClientDevice;

    roboticslab::ICartesianControl *iCartesianControl;
    roboticslab::IProximitySensors *iProximitySensors;

    double period;
    double scaling;

    bool isStopped;

    static const double SCALING_FACTOR_ON_ALERT;
};

}  // namespace roboticslab

#endif  // __STREAMING_DEVICE_CONTROLLER_HPP__

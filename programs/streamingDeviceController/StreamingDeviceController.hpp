#ifndef __STREAMING_DEVICE_CONTROLLER_HPP__
#define __STREAMING_DEVICE_CONTROLLER_HPP__

#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>

#include "StreamingDevice.hpp"
#include "CentroidTransform.hpp"

#include "ICartesianControl.h"

#ifdef SDC_WITH_SENSORS
#include "IProximitySensors.h"
#endif  // SDC_WITH_SENSORS

#define DEFAULT_DEVICE_NAME "SpaceNavigator"

#define DEFAULT_CARTESIAN_LOCAL "/streamingDevice/cartesianControlClient"
#define DEFAULT_CARTESIAN_REMOTE "/CartesianControl"

#define DEFAULT_PROXIMITY_SENSORS "/sensor_reader"
#define DEFAULT_CENTROID_LOCAL "/streamingDevice/centroid"

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
    virtual bool close();
    virtual double getPeriod();

private:
    StreamingDevice * streamingDevice;

    yarp::dev::PolyDriver cartesianControlClientDevice;
    roboticslab::ICartesianControl *iCartesianControl;

#ifdef SDC_WITH_SENSORS
    yarp::dev::PolyDriver sensorsClientDevice;
    roboticslab::IProximitySensors *iProximitySensors;

    bool disableSensorsLowLevel;
    static const double SCALING_FACTOR_ON_ALERT;
#endif  // SDC_WITH_SENSORS

    yarp::os::BufferedPort<yarp::os::Bottle> centroidPort;
    CentroidTransform centroidTransform;

    double period;
    double scaling;

    bool isStopped;
};

}  // namespace roboticslab

#endif  // __STREAMING_DEVICE_CONTROLLER_HPP__

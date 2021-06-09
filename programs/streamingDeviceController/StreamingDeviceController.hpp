#ifndef __STREAMING_DEVICE_CONTROLLER_HPP__
#define __STREAMING_DEVICE_CONTROLLER_HPP__

#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/TypedReaderCallback.h>

#include <yarp/dev/PolyDriver.h>

#include "StreamingDevice.hpp"
#include "CentroidTransform.hpp"

#include "ICartesianControl.h"

#define DEFAULT_DEVICE_NAME "SpaceNavigator"
#define DEFAULT_LOCAL_PREFIX "/streamingDeviceController"
#define DEFAULT_PERIOD 0.1  // [s]
#define DEFAULT_SCALING 10.0

#define DEFAULT_SENSOR_REMOTE "/serial/out"

#define DEFAULT_PORTMONITOR_TYPE "lua"
#define DEFAULT_PORTMONITOR_CONTEXT "sensors"
#define DEFAULT_PORTMONITOR_FILE "amor_sensors_modifier"

#define DEFAULT_THRESHOLD_ALERT_HIGH 800
#define DEFAULT_THRESHOLD_ALERT_LOW 100

namespace roboticslab
{

/**
 * @ingroup streamingDeviceController
 *
 * @brief Sends streaming commands to the cartesian controller from
 * a streaming input device like the 3Dconnexion Space Navigator.
 */
class StreamingDeviceController : public yarp::os::RFModule,
                                  public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual void onRead(yarp::os::Bottle & bot);

private:
    bool update(double timestamp);

    StreamingDevice * streamingDevice;

    yarp::dev::PolyDriver cartesianControlClientDevice;
    roboticslab::ICartesianControl *iCartesianControl;

    yarp::os::BufferedPort<yarp::os::Bottle> proximityPort;
    int thresholdAlertHigh;
    int thresholdAlertLow;
    bool disableSensorsLowLevel;

    yarp::os::BufferedPort<yarp::os::Bottle> centroidPort;
    CentroidTransform centroidTransform;

    yarp::os::BufferedPort<yarp::os::Bottle> syncPort;

    double period;
    double scaling;

    bool isStopped;
};

}  // namespace roboticslab

#endif  // __STREAMING_DEVICE_CONTROLLER_HPP__

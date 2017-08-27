#ifndef __STREAMING_DEVICE_CONTROLLER_HPP__
#define __STREAMING_DEVICE_CONTROLLER_HPP__

#include <vector>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include "ICartesianControl.h"
#include "IProximitySensors.h"

#define DEFAULT_DEVICE_PORT_LOCAL "/StreamingDeviceClient"
#define DEFAULT_DEVICE_PORT_REMOTE "/spacenavigator/mouse"

#define DEFAULT_ACTUATOR_LOCAL "/StreamingActuatorClient"
#define DEFAULT_ACTUATOR_REMOTE "/spacenavigator/buttons"

#define DEFAULT_CARTESIAN_LOCAL "/StreamingDeviceCartesianControlClient"
#define DEFAULT_CARTESIAN_REMOTE "/asibotSim/BasicCartesianControl"
#define DEFAULT_PROXIMITY_SENSORS "/sensor_reader"

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
    yarp::dev::PolyDriver actuatorClientDevice;
    yarp::dev::PolyDriver cartesianControlClientDevice;
    yarp::dev::PolyDriver sensorsClientDevice;

    yarp::dev::IAnalogSensor *iAnalogSensor;
    yarp::dev::IAnalogSensor *iAnalogSensorAct;
    roboticslab::ICartesianControl *iCartesianControl;
    roboticslab::IProximitySensors *iProximitySensors;

    double scaling;

    std::vector<bool> fixedAxes;  // 'true': disabled (fixed axis), 'false': enabled

    bool isStopped;
    int actuatorState;
};

}  // namespace roboticslab

#endif  // __STREAMING_DEVICE_CONTROLLER_HPP__

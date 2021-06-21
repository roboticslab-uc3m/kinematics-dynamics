#ifndef __STREAMING_DEVICE_CONTROLLER_HPP__
#define __STREAMING_DEVICE_CONTROLLER_HPP__

#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/TypedReaderCallback.h>

#include <yarp/dev/PolyDriver.h>

#include "StreamingDevice.hpp"
#include "CentroidTransform.hpp"

#include "ICartesianControl.h"

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
    ~StreamingDeviceController()
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;
    double getPeriod() override;
    void onRead(yarp::os::Bottle & bot) override;

private:
    bool update(double timestamp);

    StreamingDevice * streamingDevice;

    yarp::dev::PolyDriver cartesianControlClientDevice;
    roboticslab::ICartesianControl * iCartesianControl;

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

} // namespace roboticslab

#endif // __STREAMING_DEVICE_CONTROLLER_HPP__

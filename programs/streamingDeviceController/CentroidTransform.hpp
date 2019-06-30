#ifndef __CENTROID_TRANSFORM_HPP__
#define __CENTROID_TRANSFORM_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/Stamp.h>

#include "StreamingDevice.hpp"

#define ROT_FACTOR 0.1

namespace roboticslab
{

class StreamingDevice;

/**
 * @ingroup streamingDeviceController
 *
 * @brief ...
 */
class CentroidTransform
{
public:

    //! Constructor
    CentroidTransform();

    //! Register handle to device
    void registerStreamingDevice(StreamingDevice * streamingDevice)
    { this->streamingDevice = streamingDevice; }

    //! Set new permanence time
    void setPermanenceTime(double permanenceTime)
    { this->permanenceTime = permanenceTime; }

    //! Register or dismiss incoming bottle
    bool acceptBottle(yarp::os::Bottle * b);

    //! Process last stored bottle
    bool processStoredBottle() const;

private:

    StreamingDevice * streamingDevice;
    double permanenceTime;
    yarp::os::Bottle lastBottle;
    yarp::os::Stamp lastAcquisition;
};

}  // namespace roboticslab

#endif  // __CENTROID_TRANSFORM_HPP__

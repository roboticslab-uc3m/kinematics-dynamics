#ifndef __CENTROID_TRANSFORM_HPP__
#define __CENTROID_TRANSFORM_HPP__

#include <yarp/os/Bottle.h>

#include "StreamingDevice.hpp"

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

    //! Process incoming bottle
    bool processBottle(const yarp::os::Bottle & b);

private:

    StreamingDevice * streamingDevice;
};

}  // namespace roboticslab

#endif  // __CENTROID_TRANSFORM_HPP__

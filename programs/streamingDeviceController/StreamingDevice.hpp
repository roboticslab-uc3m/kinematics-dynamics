#ifndef __STREAMING_DEVICE_HPP__
#define __STREAMING_DEVICE_HPP__

#include <string>
#include <vector>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"

namespace roboticslab
{

class StreamingDevice;

/**
 * @ingroup streamingDeviceController
 *
 * @brief Factory class for creating instances of StreamingDevice.
 */
class StreamingDeviceFactory
{
public:
    /**
     * @brief Creates a new YARP device handle
     * @param deviceName Name of the device.
     * @param config List of options the YARP device should be opened with.
     * @return Pointer to an instance of a StreamingDevice.
     */
    static StreamingDevice * makeDevice(const std::string & deviceName, yarp::os::Searchable & config);

private:
    StreamingDeviceFactory();
};

/**
 * @ingroup streamingDeviceController
 *
 * @brief Abstract class for a YARP streaming device.
 */
class StreamingDevice : protected yarp::dev::PolyDriver
{
public:

    using PolyDriver::isValid;

    /**
     * @brief Constructor
     * @param config List of options the YARP device should be opened with.
     */
    StreamingDevice(yarp::os::Searchable & config);

    //! Destructor
    virtual ~StreamingDevice();

    /**
     * @brief Acquires plugin interfaces
     * @return true on success, false otherwise
     */
    virtual bool acquireInterfaces() = 0;

    /**
     * @brief Perform any custom initialization needed.
     * This method is called after the successful creation of the device
     * and once all interface handles are acquired.
     * @param usingStreamingPreset Whether the cartesian controller supports
     * streaming command presets or not.
     * @return true on success, false otherwise
     */
    virtual bool initialize(bool usingStreamingPreset)
    {
        return true;
    }

    /**
     * @brief Acquires data from remote device.
     * @return true on success, false otherwise
     */
    virtual bool acquireData() = 0;

    /**
     * @brief Performs required operations on stored data.
     * @param scaling Scaling factor applied to each data value.
     * @return true on success, false otherwise
     */
    virtual bool transformData(double scaling);

    /**
     * @brief Checks whether the device may forward acquired and
     * processed data to the controller.
     * @return true if valid, false otherwise
     */
    virtual bool hasValidMovementData() const;

    /**
     * @brief Sends movement command to the cartesian controller.
     */
    virtual void sendMovementCommand() = 0;

    /**
     * @brief Sends a movement command that would stop motion.
     */
    virtual void stopMotion() = 0;

    /**
     * @brief Stores handle to an ICartesianControl instance
     * @param iCartesianControl Handle to an ICartesianControl instance.
     */
    void setCartesianControllerHandle(ICartesianControl * iCartesianControl)
    {
        this->iCartesianControl = iCartesianControl;
    }

protected:

    ICartesianControl * iCartesianControl;

    std::vector<double> data;
    std::vector<bool> fixedAxes;

private:

    /**
     * @brief Stores vector of values representing axes that are always fixed.
     */
    void configureFixedAxes(const yarp::os::Value & v);
};

/**
 * @ingroup streamingDeviceController
 *
 * @brief Represents an invalid device
 *
 * A call to isValid() and other interface methods should yield false.
 */
class InvalidDevice : public StreamingDevice
{
public:

    //! Creates an invalid device
    InvalidDevice()
        : StreamingDevice(yarp::os::Value::getNullValue())
    {}

    virtual bool acquireInterfaces()
    {
        return false;
    }

    virtual bool acquireData()
    {
        return false;
    }

    virtual void sendMovementCommand()
    {}

    virtual void stopMotion()
    {}
};

}  // namespace roboticslab

#endif  // __STREAMING_DEVICE_HPP__

#ifndef __RATE_CONTROLLER_CONSOLE_HPP__
#define __RATE_CONTROLLER_CONSOLE_HPP__

#include <vector>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"

namespace roboticslab
{

/**
 * @ingroup rateControllerConsole
 *
 * @brief TBD
 */
class RateControllerConsole : public yarp::os::RFModule
{
public:
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual double getPeriod();

private:

    yarp::dev::PolyDriver remoteDevice;
};

}  // namespace roboticslab

#endif  // __RATE_CONTROLLER_CONSOLE_HPP__

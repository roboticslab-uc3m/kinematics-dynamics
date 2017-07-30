#ifndef __STREAMING_SPNAV_HPP__
#define __STREAMING_SPNAV_HPP__

#include <vector>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include "ICartesianControl.h"

#define DEFAULT_SPNAV_LOCAL "/StreamingSpnavClient"
#define DEFAULT_SPNAV_REMOTE "/spacenavigator/mouse"

#define DEFAULT_CARTESIAN_LOCAL "/SpnavCartesianControlClient"
#define DEFAULT_CARTESIAN_REMOTE "/asibotSim/BasicCartesianControl"

#define DEFAULT_SCALING 10.0

#define DEFAULT_FIXED_AXES "none"

namespace roboticslab
{

/**
 * @ingroup streamingSpnav
 *
 * @brief Sends streaming commands to the cartesian controller from
 * a 3D input device like the 3Dconnection Space Navigator.
 */
class StreamingSpnav : public yarp::os::RFModule
{

public:
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual double getPeriod();

private:
    yarp::dev::PolyDriver spnavClientDevice;
    yarp::dev::PolyDriver cartesianControlClientDevice;

    yarp::dev::IAnalogSensor *iAnalogSensor;
    roboticslab::ICartesianControl *iCartesianControl;

    double scaling;

    std::vector<bool> fixedAxes;  // 'true': disabled (fixed axis), 'false': enabled
};

}  // namespace roboticslab

#endif  // __STREAMING_SPNAV_HPP__

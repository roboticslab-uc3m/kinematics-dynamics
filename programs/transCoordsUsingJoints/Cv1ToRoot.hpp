// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CV1_TO_ROOT_HPP__
#define __CV1_TO_ROOT_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Matrix.h>

#include "PremultPorts.hpp"

#define DEFAULT_WATCHDOG    5       // [s]


namespace teo
{

/**
 * @ingroup cv1ToRoot
 *
 * @brief Transform Computer Vision values to root frame.
 */
class Cv1ToRoot : public yarp::os::RFModule {
    protected:
        bool updateModule();
        bool interruptModule();
        double getPeriod();
        double watchdog; // [s]

        yarp::os::Port outPort;
        PremultPorts premultPorts;

    public:
        bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace teo

#endif  // __PREMULT_H_HPP__


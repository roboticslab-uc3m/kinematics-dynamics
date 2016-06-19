// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BASIC_CARTESIAN_CONTROL_HPP__
#define __BASIC_CARTESIAN_CONTROL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>

#include <iostream> // only windows

#include "ColorDebug.hpp"

namespace teo
{

/**
 * @ingroup TeoYarp
 * \defgroup BasicCartesianControl
 *
 * @brief Contains teo::BasicCartesianControl.
 */

/**
 * @ingroup BasicCartesianControl
 * @brief The BasicCartesianControl class implements ICartesianSolver.
 */

class BasicCartesianControl : public yarp::dev::DeviceDriver {

    public:

        BasicCartesianControl() {}

        // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp--
        /** Get number of links for which the solver has been configured. */
        virtual bool getNumLinks(int* numLinks) {}

        // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

        /**
        * Open the DeviceDriver.
        * @param config is a list of parameters for the device.
        * Which parameters are effective for your device can vary.
        * See \ref dev_examples "device invocation examples".
        * If there is no example for your device,
        * you can run the "yarpdev" program with the verbose flag
        * set to probe what parameters the device is checking.
        * If that fails too,
        * you'll need to read the source code (please nag one of the
        * yarp developers to add documentation for your device).
        * @return true/false upon success/failure
        */
        virtual bool open(yarp::os::Searchable& config);

        /**
        * Close the DeviceDriver.
        * @return true/false on success/failure.
        */
        virtual bool close();

    protected:

};

}  // namespace teo

#endif  // __KDL_SOLVER_HPP__


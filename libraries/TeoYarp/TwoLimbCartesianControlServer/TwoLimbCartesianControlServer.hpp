// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TWO_LIMB_CARTESIAN_CONTROL_HPP__
#define __TWO_LIMB_CARTESIAN_CONTROL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream> // only windows

#include "ITwoLimbCartesianControl.h"

namespace roboticslab
{

/**
 * @ingroup TeoYarp
 * \defgroup TwoLimbCartesianControlServer
 *
 * @brief Contains roboticslab::TwoLimbCartesianControlServer.
 */

/**
 * @ingroup TwoLimbCartesianControlServer
 * @brief The TwoLimbCartesianControlServer class implements ITwoLimbCartesianControl server side.
 */

class TwoLimbCartesianControlServer : public yarp::dev::DeviceDriver, public yarp::os::PortReader {

    public:

        TwoLimbCartesianControlServer() {}

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

        // -------- PortReader declarations. Implementation in TwoLimbCartesianControlServer.cpp --------

        virtual bool read(yarp::os::ConnectionReader& connection);

    protected:

        yarp::os::RpcServer rpcServer;

        yarp::dev::PolyDriver cartesianControlDevice;
        roboticslab::ITwoLimbCartesianControl *iTwoLimbCartesianControl;

};

}  // namespace roboticslab

#endif  // __TWO_LIMB_CARTESIAN_CONTROL_HPP__


// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BASIC_CARTESIAN_CONTROL_HPP__
#define __BASIC_CARTESIAN_CONTROL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream> // only windows

#include "ICartesianSolver.h"
#include "ICartesianControl.h"

#include "ColorDebug.hpp"

#define DEFAULT_SOLVER "KdlSolver"
#define DEFAULT_ROBOT "remote_controlboard"
#define DEFAULT_INIT_STATE VOCAB_CC_NOT_CONTROLLING

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

class BasicCartesianControl : public yarp::dev::DeviceDriver, public ICartesianControl {

    public:

        BasicCartesianControl() : currentState(DEFAULT_INIT_STATE){}

        // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp--
        /** Inform on control state, and get robot position and perform forward kinematics. */
        virtual bool stat(int &state, std::vector<double> &x);

        /** Perform inverse kinematics (using robot position as initial guess) but do not move. */
        virtual bool inv(const std::vector<double> &xd, std::vector<double> &q);

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

        yarp::dev::PolyDriver solverDevice;
        teo::ICartesianSolver *iCartesianSolver;

        yarp::dev::PolyDriver robotDevice;
        yarp::dev::IEncoders *iEncoders;
        yarp::dev::IVelocityControl *iVelocityControl;
        yarp::dev::IPositionControl *iPositionControl;

        int numRobotJoints;

        int currentState;
};

}  // namespace teo

#endif  // __KDL_SOLVER_HPP__


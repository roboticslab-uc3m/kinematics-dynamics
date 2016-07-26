// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BASIC_TWO_LIMB_CARTESIAN_CONTROL_HPP__
#define __BASIC_TWO_LIMB_CARTESIAN_CONTROL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream> // only windows

#include <math.h>  //-- fabs

#include "ICartesianSolver.h"
#include "ITwoLimbCartesianControl.h"

#include "LineTrajectory.hpp"

#include "ColorDebug.hpp"

#define DEFAULT_SOLVER "KdlSolver"
#define DEFAULT_ROBOT "remote_controlboard"
#define DEFAULT_INIT_STATE VOCAB_CC_NOT_CONTROLLING
#define DEFAULT_MS 50
#define MAX_ANG_VEL 7.5
#define DEFAULT_GAIN 1.0

namespace teo
{

/**
 * @ingroup TeoYarp
 * \defgroup BasicTwoLimbCartesianControl
 *
 * @brief Contains teo::BasicTwoLimbCartesianControl.
 */

/**
 * @ingroup BasicTwoLimbCartesianControl
 * @brief The BasicTwoLimbCartesianControl class implements ICartesianSolver.
 */

class BasicTwoLimbCartesianControl : public yarp::dev::DeviceDriver, public ITwoLimbCartesianControl, public yarp::os::RateThread {

    public:

        BasicTwoLimbCartesianControl() : currentState(DEFAULT_INIT_STATE), RateThread(DEFAULT_MS) {}

        // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp--
        /** Inform on control state, and get robot position and perform forward kinematics. */
        virtual bool stat(int &state, std::vector<double> &x);

        /** step */
        virtual bool step(const std::vector<double> &xd);

        /** stop */
        virtual bool stopControl();

        // -------- RateThread declarations. Implementation in RateThreadImpl.cpp --------

        /** Loop function. This is the thread itself. */
        virtual void run();

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
        yarp::dev::IPositionControl *iPositionControl;
        yarp::dev::IVelocityControl *iVelocityControl;
        yarp::dev::IControlLimits *iControlLimits;
        yarp::dev::ITorqueControl *iTorqueControl;

        int numRobotJoints, numSolverLinks;

        /** State encoded as a VOCAB which can be stored as an int */
        int currentState;

        int getCurrentState();
        void setCurrentState(int value);
        yarp::os::Semaphore currentStateReady;

        /** STEP keep track of movement start time to know at what time of trajectory movement we are */
        double movementStartTime;

        /** STEP store Cartesian trajectory */
        LineTrajectory trajectory;

        /** STEP desired Cartesian velocity */
        std::vector<double> xdotd;

};

}  // namespace teo

#endif  // __BASIC_TWO_LIMB_CARTESIAN_CONTROL_HPP__

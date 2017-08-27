// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BASIC_CARTESIAN_CONTROL_HPP__
#define __BASIC_CARTESIAN_CONTROL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream> // only windows
#include <vector>

#include "ICartesianSolver.h"
#include "ICartesianControl.h"

#include "LineTrajectory.hpp"

#define DEFAULT_SOLVER "KdlSolver"
#define DEFAULT_ROBOT "remote_controlboard"
#define DEFAULT_INIT_STATE VOCAB_CC_NOT_CONTROLLING
#define DEFAULT_MS 50
#define MAX_ANG_VEL 7.5
#define DEFAULT_GAIN 0.05
#define DEFAULT_QDOT_LIMIT 10

namespace roboticslab
{

/**
 * @ingroup TeoYarp
 * \defgroup BasicCartesianControl
 *
 * @brief Contains roboticslab::BasicCartesianControl.

@section BasicCartesianControl_Running1 Example with a Fake robot

First we must run a YARP name server if it is not running in our current namespace:

\verbatim
[on terminal 1] yarp server
\endverbatim

And then launch the actual library:

\verbatim
[on terminal 2] yarpdev --device BasicCartesianControl --robot FakeControlboard --angleRepr axisAngle --link_0 "(A 1)"
\endverbatim

Along the output, observe a line like the following.
\verbatim
[info] DeviceDriverImpl.cpp:26 open(): Using angleRepr: axisAngle.
\endverbatim
This would mean we are using an axis/angle notation (par de rotación). Note that it is a variable parameter; in fact, we have forced it above.

We connect, can ask for help, etc. Here's an example interaction:
\verbatim
[on terminal 3] yarp rpc /CartesianControl/rpc_transform:s
[>>] help
Response: [stat] [inv] [movj] [movl] [movv] [gcmp] [forc] [stop]
[>>] stat
Response: [ccnc] 1.0 0.0 0.0 0.0 0.0 1.0 0.0
\endverbatim
Stat returns the controller status, and forward kinematics.
[ccnc] means Cartesian Control Not Controlling (also note that notation is in constant evolution, so put an issue if it's not updated!).
The first 3 parameters of the forward kinnematics are X,Y,Z of the end-effector. The other parameters correspond to orientation in the previously
set notation, which in this example is axis/angle, so this would read: on Z, 0 degrees.

Let's now move to a reachable position/orientation.
\verbatim
[>>] movj 1.0 0.0 0.0 0.0 0.0 1.0 0.0
Response: [ok]
\endverbatim

@section BasicCartesianControl_Running2 Example with teoSim

What moslty changes is the library command line invocation. We also change the server port name. The following is an example for the simulated robot's right arm.
\verbatim
[on terminal 2] yarpdev --device BasicCartesianControl --name /teoSim/rightArm/CartesianControl --from /usr/local/share/teo-configuration-files/contexts/kinematics/rightArmKinematics.ini --angleRepr axisAngle --robot remote_controlboard --local /BasicCartesianControl/teoSim/rightArm --remote /teoSim/rightArm
[on terminal 3] yarp rpc /teoSim/rightArm/CartesianControl/rpc_transform:s
\endverbatim


@section BasicCartesianControl_Running3 Example with real TEO

What moslty changes is the library command line invocation. We also change the server port name. The following is an example for the robot's right arm.
\verbatim
[on terminal 2] yarpdev --device BasicCartesianControl --name /teo/rightArm/CartesianControl  --from /usr/local/share/teo-configuration-files/contexts/kinematics/rightArmKinematics.ini --angleRepr axisAngle --robot remote_controlboard --local /BasicCartesianControl/teo/rightArm --remote /teo/rightArm
[on terminal 3] yarp rpc /teo/rightArm/CartesianControl/rpc:s
\endverbatim
On the real robot, you can even activate Gravity Compensation (warning: dangerous if kinematic/dynamic model has not been reviewed!).
\verbatim
[>>] gcmp
\endverbatim

@section BasicCartesianControl_Running4 Very Important

When you launch the BasicCartesianControl device as in [terminal 2], it's actually wrapped: CartesianControlServer is the device that is
actually loaded, and BasicCartesianControl becomes its subdevice. The server is what allows us to interact via the YARP RPC port mechanism.
If you are creating a C++ program, you do not have to sned these raw port commands. Use CartesianControlClient instead, because the server and client
are YARP devices (further reading on why this is good: <a href="http://asrob.uc3m.es/index.php/Tutorial_yarp_devices#.C2.BFQu.C3.A9_m.C3.A1s_nos_permiten_los_YARP_device.3F">here (spanish)</a>).

 */

/**
 * @ingroup BasicCartesianControl
 * @brief The BasicCartesianControl class implements ICartesianControl.
 */

class BasicCartesianControl : public yarp::dev::DeviceDriver, public ICartesianControl, public yarp::os::RateThread
{

    public:

        BasicCartesianControl() : currentState(DEFAULT_INIT_STATE), RateThread(DEFAULT_MS) {}

        // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp--

        virtual bool stat(int &state, std::vector<double> &x);

        virtual bool inv(const std::vector<double> &xd, std::vector<double> &q);

        virtual bool movj(const std::vector<double> &xd);

        virtual bool relj(const std::vector<double> &xd);

        virtual bool movl(const std::vector<double> &xd);

        virtual bool movv(const std::vector<double> &xdotd);

        virtual bool gcmp();

        virtual bool forc(const std::vector<double> &td);

        virtual bool stopControl();

        virtual bool tool(const std::vector<double> &x);

        /** fwd */
        virtual bool fwd(const std::vector<double> &rot);

        /** bkwd*/
        virtual bool bkwd(const std::vector<double> &rot);

        /** rot */
        virtual bool rot(const std::vector<double> &rot);

        /** vmos */
        virtual bool vmos(const std::vector<double> &xdot);

        /** pose */
        virtual bool pose(const std::vector<double> &x);

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
        roboticslab::ICartesianSolver *iCartesianSolver;

        yarp::dev::PolyDriver robotDevice;
        yarp::dev::IEncoders *iEncoders;
        yarp::dev::IPositionControl *iPositionControl;
        yarp::dev::IVelocityControl *iVelocityControl;
        yarp::dev::IControlLimits *iControlLimits;
        yarp::dev::ITorqueControl *iTorqueControl;
        yarp::dev::IControlMode *iControlMode;

        int numRobotJoints, numSolverJoints;

        /** State encoded as a VOCAB which can be stored as an int */
        int currentState;

        int getCurrentState();
        void setCurrentState(int value);
        yarp::os::Semaphore currentStateReady;

        /** MOVL keep track of movement start time to know at what time of trajectory movement we are */
        double movementStartTime;

        /** MOVL store Cartesian trajectory */
        LineTrajectory trajectory;

        /** MOVV desired Cartesian velocity */
        std::vector<double> xdotd;

        /** FORC desired Cartesian force */
        std::vector<double> td;
};

}  // namespace roboticslab

#endif  // __BASIC_CARTESIAN_CONTROL_HPP__

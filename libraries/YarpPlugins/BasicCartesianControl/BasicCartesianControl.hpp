// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BASIC_CARTESIAN_CONTROL_HPP__
#define __BASIC_CARTESIAN_CONTROL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PreciselyTimed.h>

#include <iostream> // only windows
#include <vector>

#include "ICartesianSolver.h"
#include "ICartesianControl.h"

#include "ICartesianTrajectory.hpp"

#define DEFAULT_SOLVER "KdlSolver"
#define DEFAULT_ROBOT "remote_controlboard"
#define DEFAULT_INIT_STATE VOCAB_CC_NOT_CONTROLLING
#define DEFAULT_GAIN 0.05
#define DEFAULT_QDOT_LIMIT 10.0
#define DEFAULT_DURATION 10.0
#define DEFAULT_CMC_PERIOD_MS 50
#define DEFAULT_WAIT_PERIOD_MS 30
#define DEFAULT_REFERENCE_FRAME "base"
#define DEFAULT_STREAMING_PRESET 0

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
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
class BasicCartesianControl : public yarp::dev::DeviceDriver,
                              public yarp::os::PeriodicThread,
                              public ICartesianControl
{
public:

    BasicCartesianControl() : yarp::os::PeriodicThread(DEFAULT_CMC_PERIOD_MS * 0.001),
                              iCartesianSolver(NULL),
                              iEncoders(NULL),
                              iPositionControl(NULL),
                              iPositionDirect(NULL),
                              iVelocityControl(NULL),
                              iControlLimits(NULL),
                              iTorqueControl(NULL),
                              iControlMode(NULL),
                              iPreciselyTimed(NULL),
                              referenceFrame(ICartesianSolver::BASE_FRAME),
                              gain(DEFAULT_GAIN),
                              maxJointVelocity(DEFAULT_QDOT_LIMIT),
                              duration(DEFAULT_DURATION),
                              cmcPeriodMs(DEFAULT_CMC_PERIOD_MS),
                              waitPeriodMs(DEFAULT_WAIT_PERIOD_MS),
                              numRobotJoints(0),
                              numSolverJoints(0),
                              currentState(DEFAULT_INIT_STATE),
                              streamingCommand(DEFAULT_STREAMING_PRESET),
                              movementStartTime(0),
                              iCartesianTrajectory(NULL),
                              cmcSuccess(true)
    {}

    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp--

    virtual bool stat(std::vector<double> &x, int * state = 0, double * timestamp = 0);

    virtual bool inv(const std::vector<double> &xd, std::vector<double> &q);

    virtual bool movj(const std::vector<double> &xd);

    virtual bool relj(const std::vector<double> &xd);

    virtual bool movl(const std::vector<double> &xd);

    virtual bool movv(const std::vector<double> &xdotd);

    virtual bool gcmp();

    virtual bool forc(const std::vector<double> &td);

    virtual bool stopControl();

    virtual bool wait(double timeout);

    virtual bool tool(const std::vector<double> &x);

    virtual void twist(const std::vector<double> &xdot);

    virtual void pose(const std::vector<double> &x, double interval);

    virtual void movi(const std::vector<double> &x);

    virtual bool setParameter(int vocab, double value);

    virtual bool getParameter(int vocab, double * value);

    virtual bool setParameters(const std::map<int, double> & params);

    virtual bool getParameters(std::map<int, double> & params);

    // -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------

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

    int getCurrentState() const;
    void setCurrentState(int value);

    bool checkJointLimits(const std::vector<double> &q);
    bool checkJointLimits(const std::vector<double> &q, const std::vector<double> &qd);
    bool checkJointVelocities(const std::vector<double> &qdot);

    bool setControlModes(int mode);
    bool presetStreamingCommand(int command);

    void handleMovj(const std::vector<double> &q);
    void handleMovl(const std::vector<double> &q);
    void handleMovv(const std::vector<double> &q);
    void handleGcmp(const std::vector<double> &q);
    void handleForc(const std::vector<double> &q);

    yarp::dev::PolyDriver solverDevice;
    roboticslab::ICartesianSolver *iCartesianSolver;

    yarp::dev::PolyDriver robotDevice;
    yarp::dev::IEncoders *iEncoders;
    yarp::dev::IPositionControl *iPositionControl;
    yarp::dev::IPositionDirect * iPositionDirect;
    yarp::dev::IVelocityControl *iVelocityControl;
    yarp::dev::IControlLimits *iControlLimits;
    yarp::dev::ITorqueControl *iTorqueControl;
    yarp::dev::IControlMode *iControlMode;
    yarp::dev::IPreciselyTimed *iPreciselyTimed;

    ICartesianSolver::reference_frame referenceFrame;

    double gain;
    double maxJointVelocity;
    double duration; // [s]

    int cmcPeriodMs;
    int waitPeriodMs;
    int numRobotJoints, numSolverJoints;

    /** State encoded as a VOCAB which can be stored as an int */
    int currentState;

    int streamingCommand;

    mutable yarp::os::Semaphore currentStateReady;
    mutable yarp::os::Semaphore trajectoryMutex;

    /** MOVJ store previous reference speeds */
    std::vector<double> vmoStored;

    /** MOVL keep track of movement start time to know at what time of trajectory movement we are */
    double movementStartTime;

    /** MOVL store Cartesian trajectory */
    ICartesianTrajectory* iCartesianTrajectory;

    /** FORC desired Cartesian force */
    std::vector<double> td;

    bool cmcSuccess;

    std::vector<double> qMin, qMax;
};

}  // namespace roboticslab

#endif  // __BASIC_CARTESIAN_CONTROL_HPP__

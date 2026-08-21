// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BASIC_CARTESIAN_CONTROL_HPP__
#define __BASIC_CARTESIAN_CONTROL_HPP__

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <yarp/conf/version.h>

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IPreciselyTimed.h>

#include <kdl/trajectory.hpp>

#include "ICartesianSolver.h"
#include "ICartesianControl.h"
#include "BasicCartesianControl_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup BasicCartesianControl
 *
 * @brief Contains BasicCartesianControl.

@section BasicCartesianControl_Running1 Example with a Fake robot

First we must run a YARP name server if it is not running in our current namespace:

\verbatim
[on terminal 1] yarp server
\endverbatim

And then launch the actual library:

\verbatim
[on terminal 2] yarpdev --device BasicCartesianControl --robot EmulatedControlBoard --angleRepr axisAngle --link_0 "(A 1)"
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
                              public roboticslab::ICartesianControl,
                              public BasicCartesianControl_ParamsParser
{
public:
    BasicCartesianControl() : yarp::os::PeriodicThread(1.0, yarp::os::PeriodicThreadClock::Absolute)
    {}

    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --

    // RPC commands
    yarp::dev::ReturnValue getState(std::vector<double> & x, roboticslab::ICartesianControl::State * state = nullptr, double * timestamp = nullptr) override;
    yarp::dev::ReturnValue solvePose(const std::vector<double> & xd, std::vector<double> & q) override;
    yarp::dev::ReturnValue moveJoint(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveLinear(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveVelocity(const std::vector<double> & xdotd) override;
    yarp::dev::ReturnValue gravityCompensation() override;
    yarp::dev::ReturnValue forceControl(const std::vector<double> & fd) override;
    yarp::dev::ReturnValue stopControl() override;
    yarp::dev::ReturnValue wait(double timeout) override;
    yarp::dev::ReturnValue changeTool(const std::vector<double> & x) override;
    yarp::dev::ReturnValue actuateTool(roboticslab::ICartesianControl::Actuator command) override;

    // streaming commands
    void pose(const std::vector<double> & x) override;
    void twist(const std::vector<double> & xdot) override;
    void wrench(const std::vector<double> & w) override;

    // configuration getters/setters
    yarp::dev::ReturnValue setParameter(roboticslab::ICartesianControl::Config vocab, double value) override;
    yarp::dev::ReturnValue getParameter(roboticslab::ICartesianControl::Config vocab, double * value) override;
    yarp::dev::ReturnValue setParameters(const std::map<roboticslab::ICartesianControl::Config, double> & params) override;
    yarp::dev::ReturnValue getParameters(std::map<roboticslab::ICartesianControl::Config, double> & params) override;

    // -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------
    void run() override;

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    class StateWatcher
    {
    public:
        template <typename Fn>
        StateWatcher(Fn && fn) : handler(std::move(fn))
        {}

        ~StateWatcher()
        { if (handler) handler(); }

        void suppress() const
        { handler = nullptr; }

    private:
        mutable std::function<void()> handler;
    };

    roboticslab::ICartesianControl::State getCurrentState() const;
    void setCurrentState(roboticslab::ICartesianControl::State value);

    bool checkJointLimits(const std::vector<double> & q);
    bool checkJointLimits(const std::vector<double> & q, const std::vector<double> & qdot);
    bool checkJointVelocities(const std::vector<double> & qdot);
    bool doFailFastChecks(const std::vector<double> & initialQ);
    bool checkControlModes(int mode);
    bool setControlModes(int mode);
    bool presetStreamingCommand(roboticslab::ICartesianControl::Streaming command);
    bool computeIsocronousSpeeds(const std::vector<double> & q, const std::vector<double> & qd, std::vector<double> & qdot);

    void handleMovj(const std::vector<double> & q, const StateWatcher & watcher);
    void handleMovlVel(const std::vector<double> & q, const StateWatcher & watcher);
    void handleMovlPosd(const std::vector<double> & q, const StateWatcher & watcher);
    void handleMovv(const std::vector<double> & q, const StateWatcher & watcher);
    void handleGcmp(const std::vector<double> & q, const StateWatcher & watcher);
    void handleForc(const std::vector<double> & q, const std::vector<double> & qdot, const std::vector<double> & qdotdot, const StateWatcher & watcher);

    yarp::dev::PolyDriver solverDevice;
    roboticslab::ICartesianSolver * iCartesianSolver {nullptr};

    yarp::dev::PolyDriver robotDevice;
    yarp::dev::IControlMode * iControlMode {nullptr};
    yarp::dev::IEncoders * iEncoders {nullptr};
    yarp::dev::IPositionControl * iPositionControl {nullptr};
    yarp::dev::IPositionDirect *  iPositionDirect {nullptr};
    yarp::dev::IPreciselyTimed * iPreciselyTimed {nullptr};
    yarp::dev::ITorqueControl * iTorqueControl {nullptr};
    yarp::dev::IVelocityControl * iVelocityControl {nullptr};

    roboticslab::ICartesianSolver::Frame referenceFrame;

#if YARP_VERSION_COMPARE(>=, 4,0,0)
    std::size_t numJoints {0};
#else
    int numJoints {0};
#endif
    roboticslab::ICartesianControl::State currentState {roboticslab::ICartesianControl::State::NONE};
    roboticslab::ICartesianControl::Streaming streamingCommand {roboticslab::ICartesianControl::Streaming::POSE};

    mutable std::mutex stateMutex;

    /** MOVEJ store previous reference speeds */
    std::vector<double> vmoStored;

    /** MOVEL keep track of movement start time to know at what time of trajectory movement we are */
    double movementStartTime {0.0};

    /** MOVEL store Cartesian trajectory */
    std::vector<std::unique_ptr<KDL::Trajectory>> trajectories;

    /** FORCE desired Cartesian force */
    std::vector<double> fd;

    int encoderErrors {0};
    std::atomic<bool> cmcSuccess {true};

    std::vector<double> qMin, qMax;
    std::vector<double> qdotMin, qdotMax;
    std::vector<double> qRefSpeeds;
};

#endif // __BASIC_CARTESIAN_CONTROL_HPP__

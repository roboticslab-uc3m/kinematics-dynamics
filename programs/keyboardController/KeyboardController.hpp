#ifndef __KEYBOARD_CONTROLLER_HPP__
#define __KEYBOARD_CONTROLLER_HPP__

#include <string>
#include <vector>
#include <functional>

#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IVelocityControl.h>

#include "ICartesianControl.h"
#include "KinematicRepresentation.hpp"

#define DEFAULT_ROBOT_LOCAL "/KeyboardControllerClient"
#define DEFAULT_ROBOT_REMOTE "/asibot/asibotManipulator"

#define DEFAULT_CARTESIAN_LOCAL "/KeyboardCartesianControlClient"
#define DEFAULT_CARTESIAN_REMOTE "/asibotSim/BasicCartesianControl"

#define DEFAULT_ANGLE_REPR "axisAngle" // keep in sync with KinRepresentation::parseEnumerator's
                                       // fallback in ::open()

#define CMC_RATE_MS 10

namespace roboticslab
{

/**
 * @ingroup keyboardController
 *
 * @brief Helper thread for sending streaming cartesian
 * commands to the controller.
 */
class KeyboardRateThread : public yarp::os::RateThread
{
public:
    typedef std::vector<double> data_type;
    typedef void (ICartesianControl::*cart_command)(const data_type &);

    KeyboardRateThread(roboticslab::ICartesianControl * iCartesianControl)
        : yarp::os::RateThread(CMC_RATE_MS),
          iCartesianControl(iCartesianControl),
          currentCommand(&ICartesianControl::vmos)
    {}

    virtual void run()
    {
        (iCartesianControl->*currentCommand)(currentData);
    }

    void setCurrentCommand(cart_command cmd)
    {
        currentCommand = cmd;
    }

    void setCurrentData(const data_type & data)
    {
        currentData = data;
    }

    void beforeStart()
    {
        // prevents execution of first run() step after start()
        suspend();
    }

private:
    roboticslab::ICartesianControl * iCartesianControl;
    cart_command currentCommand;
    data_type currentData;
};

/**
 * @ingroup keyboardController
 *
 * @brief Sends streaming commands to the cartesian controller from
 * a standard keyboard.
 *
 * Uses the terminal as a simple user interface. Also accepts joint commands
 * if connected to a remote controlboard.
 */
class KeyboardController : public yarp::os::RFModule
{
public:
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual double getPeriod();
    virtual bool close();

private:
    // used for array indexes and size checks
    enum joint { Q1 = 0, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, MAX_JOINTS };
    enum cart { X = 0, Y, Z, ROTX, ROTY, ROTZ, NUM_CART_COORDS };

    enum cart_frames { INERTIAL, END_EFFECTOR };

    enum control_modes { NOT_CONTROLLING, JOINT_MODE, CARTESIAN_MODE };

    static const std::plus<double> increment_functor;
    static const std::minus<double> decrement_functor;

    template <typename func>
    void incrementOrDecrementJointVelocity(joint q, func op);

    template <typename func>
    void incrementOrDecrementCartesianVelocity(cart coord, func op);

    void toggleReferenceFrame();

    void actuateTool(int command);

    void printJointPositions();
    void printCartesianPositions();

    void issueStop();

    void printHelp();

    int axes;
    int currentActuatorCommand;

    cart_frames cartFrame;
    std::string angleRepr;
    KinRepresentation::orientation_system orient;
    control_modes controlMode;

    yarp::dev::PolyDriver controlboardDevice;
    yarp::dev::PolyDriver cartesianControlDevice;

    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IControlLimits2 * iControlLimits;
    yarp::dev::IVelocityControl * iVelocityControl;

    roboticslab::ICartesianControl * iCartesianControl;

    KeyboardRateThread * cartesianThread;

    std::vector<double> maxVelocityLimits;
    std::vector<double> currentJointVels;
    std::vector<double> currentCartVels;

    static const std::vector<double> ZERO_CARTESIAN_VELOCITY;

    static const double JOINT_VELOCITY_STEP;
    static const double CARTESIAN_LINEAR_VELOCITY_STEP;
    static const double CARTESIAN_ANGULAR_VELOCITY_STEP;
};

}  // namespace roboticslab

#endif  // __KEYBOARD_CONTROLLER_HPP__

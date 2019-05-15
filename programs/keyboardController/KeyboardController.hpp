// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KEYBOARD_CONTROLLER_HPP__
#define __KEYBOARD_CONTROLLER_HPP__

#include <string>
#include <vector>
#include <functional>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IVelocityControl.h>

#include <yarp/conf/version.h>

#if YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR >= 1
# include <yarp/dev/IControlLimits.h>
#else
# include <yarp/dev/IControlLimits2.h>
#endif // YARP_VERSION_MAJOR == 3 && YARP_VERSION_MINOR == 0

#include "LinearTrajectoryThread.hpp"
#include "ICartesianControl.h"
#include "KinematicRepresentation.hpp"

#define DEFAULT_ROBOT_LOCAL "/KeyboardControllerClient"
#define DEFAULT_ROBOT_REMOTE "/asibot/asibotManipulator"

#define DEFAULT_CARTESIAN_LOCAL "/KeyboardCartesianControlClient"
#define DEFAULT_CARTESIAN_REMOTE "/asibotSim/BasicCartesianControl"

#define DEFAULT_ANGLE_REPR "axisAngle" // keep in sync with KinRepresentation::parseEnumerator's
                                       // fallback in ::open()

#define DEFAULT_THREAD_MS 50

namespace roboticslab
{

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

    enum control_modes { NOT_CONTROLLING, JOINT_MODE, CARTESIAN_MODE };

    static const std::plus<double> increment_functor;
    static const std::minus<double> decrement_functor;

    template <typename func>
    void incrementOrDecrementJointVelocity(joint q, func op);

    template <typename func>
    void incrementOrDecrementCartesianVelocity(cart coord, func op);

    void toggleReferenceFrame();

    void printJointPositions();
    void printCartesianPositions();

    void issueStop();

    void printHelp();

    int axes;

    ICartesianSolver::reference_frame cartFrame;
    std::string angleRepr;
    KinRepresentation::orientation_system orient;
    control_modes controlMode;

    bool usingThread;
    LinearTrajectoryThread * linTrajThread;

    yarp::dev::PolyDriver controlboardDevice;
    yarp::dev::PolyDriver cartesianControlDevice;

    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IControlLimits * iControlLimits;
    yarp::dev::IVelocityControl * iVelocityControl;

    roboticslab::ICartesianControl * iCartesianControl;

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

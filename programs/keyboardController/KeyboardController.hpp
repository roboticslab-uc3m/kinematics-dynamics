// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KEYBOARD_CONTROLLER_HPP__
#define __KEYBOARD_CONTROLLER_HPP__

#include <string>
#include <vector>
#include <functional>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IVelocityControl.h>

#include "LinearTrajectoryThread.hpp"
#include "ICartesianControl.h"
#include "KinematicRepresentation.hpp"

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
    ~KeyboardController() override
    { close(); }

    // used for array indexes and size checks
    enum joint { Q1 = 0, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, MAX_JOINTS };
    enum cart { X = 0, Y, Z, ROTX, ROTY, ROTZ, NUM_CART_COORDS };

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    double getPeriod() override;
    bool close() override;

private:
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

    int axes {0};
    int currentActuatorCommand {VOCAB_CC_ACTUATOR_NONE};

    ICartesianSolver::reference_frame cartFrame {ICartesianSolver::BASE_FRAME};
    std::string angleRepr;
    KinRepresentation::orientation_system orient {KinRepresentation::orientation_system::AXIS_ANGLE};
    control_modes controlMode {NOT_CONTROLLING};

    bool usingThread {false};
    LinearTrajectoryThread * linTrajThread {nullptr};

    yarp::dev::PolyDriver controlBoardDevice;
    yarp::dev::PolyDriver cartesianControlDevice;

    yarp::dev::IEncoders * iEncoders {nullptr};
    yarp::dev::IControlMode * iControlMode {nullptr};
    yarp::dev::IControlLimits * iControlLimits {nullptr};
    yarp::dev::IVelocityControl * iVelocityControl {nullptr};

    roboticslab::ICartesianControl * iCartesianControl {nullptr};

    std::vector<double> maxVelocityLimits;
    std::vector<double> currentJointVels;
    std::vector<double> currentCartVels;
};

} // namespace roboticslab

#endif // __KEYBOARD_CONTROLLER_HPP__

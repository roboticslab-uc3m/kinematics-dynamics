namespace yarp roboticslab

struct yReturnValue {
} (
    yarp.name = "yarp::dev::ReturnValue"
    yarp.includefile = "yarp/dev/ReturnValue.h"
)

enum CartesianControlMode {
} (
    yarp.name = "roboticslab::ICartesianControl::Mode"
    yarp.includefile = "ICartesianControl.h"
    yarp.enumbase = "int32_t"
)

enum CartesianControlConfig {
} (
    yarp.name = "roboticslab::ICartesianControl::Config"
    yarp.includefile = "ICartesianControl.h"
    yarp.enumbase = "int32_t"
)

enum CartesianControlActuator {
} (
    yarp.name = "roboticslab::ICartesianControl::Actuator"
    yarp.includefile = "ICartesianControl.h"
    yarp.enumbase = "int32_t"
)

// Cannot wrap ICartesianControl::ControllerState because it is not a
// yarp::os::Portable (not doable, must live in a header-only CMake target).
struct return_get_state {
    1: yReturnValue ret;
    2: list<double> x;
    3: CartesianControlMode mode;
    4: double timestamp;
    5: double duration;
    6: double progress;
    7: bool success;
}

struct return_solve_pose {
    1: yReturnValue ret;
    2: list<double> q;
}

struct return_get_parameter_double {
    1: yReturnValue ret;
    2: double value;
}

struct return_get_parameter_vocab {
    1: yReturnValue ret;
    2: i32 value;
}

struct return_get_parameters {
    1: yReturnValue ret;
    2: map<CartesianControlConfig, double> params;
}

service CartesianControlMsgs
{
    /**
     * Inform on control state, get robot position and perform forward kinematics.
     */
    return_get_state getState();

    /**
     * Perform inverse kinematics (using robot position as initial guess), but do not move.
     */
    return_solve_pose solvePose(1: list<double> xd);

    /**
     * Move in joint space, absolute coordinates.
     */
    yReturnValue moveJoint(1: list<double> xd);

    /**
     * Linear move to target position.
     */
    yReturnValue moveLinear(1: list<double> xd);

    /**
     * Linear move with given velocity.
     */
    yReturnValue moveVelocity(1: list<double> xdotd);

    /**
     * Enable gravity compensation.
     */
    yReturnValue gravityCompensation();

    /**
     * Force control.
     */
    yReturnValue forceControl(1: list<double> fd);

    /**
     * Stop control.
     */
    yReturnValue stopControl();

    /**
     * Change tool.
     */
    yReturnValue changeTool(1: list<double> x);

    /**
     * Actuate tool.
     */
    yReturnValue actuateTool(1: CartesianControlActuator command);

    /**
     * Set a configuration parameter of double type.
     */
    yReturnValue setParameterDouble(1: CartesianControlConfig vocab, 2: double value);

    /**
     * Set a configuration parameter of vocab type.
     */
    yReturnValue setParameterVocab(1: CartesianControlConfig vocab, 2: i32 value);

    /**
     * Retrieve a configuration parameter of double type.
     */
    return_get_parameter_double getParameterDouble(1: CartesianControlConfig vocab);

    /**
     * Retrieve a configuration parameter of vocab type.
     */
    return_get_parameter_vocab getParameterVocab(1: CartesianControlConfig vocab);

    /**
     * Set multiple configuration parameters.
     */
    yReturnValue setParameters(1: map<CartesianControlConfig, double> params);

    /**
     * Retrieve multiple configuration parameters.
     */
    return_get_parameters getParameters();
}

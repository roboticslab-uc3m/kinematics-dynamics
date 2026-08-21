namespace yarp roboticslab

struct yReturnValue {
} (
    yarp.name = "yarp::dev::ReturnValue"
    yarp.includefile = "yarp/dev/ReturnValue.h"
)

enum CartesianControlState {
} (
    yarp.name = "roboticslab::ICartesianControl::State"
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

struct return_get_state {
    1: yReturnValue ret;
    2: list<double> x;
    3: CartesianControlState state;
    4: double timestamp;
}

struct return_solve_pose {
    1: yReturnValue ret;
    2: list<double> q;
}

struct return_get_parameter {
    1: yReturnValue ret;
    2: double value;
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
     * Set a configuration parameter.
     */
    yReturnValue setParameter(1: CartesianControlConfig vocab, 2: double value);

    /**
     * Retrieve a configuration parameter.
     */
    return_get_parameter getParameter(1: CartesianControlConfig vocab);

    /**
     * Set multiple configuration parameters.
     */
    yReturnValue setParameters(1: map<CartesianControlConfig, double> params);

    /**
     * Retrieve multiple configuration parameters.
     */
    return_get_parameters getParameters();
}

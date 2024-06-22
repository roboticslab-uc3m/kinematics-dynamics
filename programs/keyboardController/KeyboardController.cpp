// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KeyboardController.hpp"

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include <cstdlib>
#include <csignal>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_LOCAL_PREFIX = "/keyboardController";

constexpr auto DEFAULT_ANGLE_REPR = "axisAngle"; // keep in sync with KinRepresentation::parseEnumerator's
                                                 // fallback in ::open()

constexpr auto DEFAULT_THREAD_MS = 50;

constexpr auto DEFAULT_JOINT_MODE = "vel";

constexpr auto DEFAULT_JOINT_POSITION_STEP = 1.0; // [deg]
constexpr auto DEFAULT_JOINT_VELOCITY_STEP = 0.5; // [deg/s]

constexpr auto CARTESIAN_LINEAR_VELOCITY_STEP = 0.005; // [m]
constexpr auto CARTESIAN_ANGULAR_VELOCITY_STEP = 0.01; // [deg]

const std::vector<double> ZERO_CARTESIAN_VELOCITY(KeyboardController::NUM_CART_COORDS);

namespace
{
    struct termios ots;

    inline bool readKey(char * key)
    {
        return read(STDIN_FILENO, key, 1) > 0;
    }

    std::ostream & operator<<(std::ostream & out, const std::vector<double> & v)
    {
        out << '[';

        for (auto it = v.begin(); it != v.end(); ++it)
        {
            out << *it;

            if (std::next(it) != v.end())
            {
                out << ", ";
            }
        }

        out << "]";
        return out;
    }

    std::vector<double> roundZeroes(const std::vector<double> & v_in)
    {
        static constexpr double precision = 1e-6;

        auto v_out(v_in);

        for (auto it = v_out.begin(); it != v_out.end(); ++it)
        {
            if (std::abs(*it) < precision)
            {
                *it = 0.0;
            }
        }

        return v_out;
    }

    // reset the TTY configurations that was changed in the ttyset function (UNIX)
    inline void ttyreset(int signal)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &ots);
        tcsetattr(STDOUT_FILENO, TCSANOW, &ots);
    }

    // configure the TTY for reading keyboard input (UNIX)
    void ttyset()
    {
        struct termios ts;
        tcgetattr(STDIN_FILENO, &ts);
        ots = ts;

        ts.c_lflag &= ~ICANON;  // raw data mode
        ts.c_lflag &= ~(ECHO | ECHOCTL | ECHONL);  // no echo
        ts.c_lflag |= IEXTEN;

        tcsetattr(STDIN_FILENO, TCSANOW, &ts);  // set raw data mode

        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);  // make stdin non blocking
        fcntl(STDOUT_FILENO, F_SETFL, fcntl(STDOUT_FILENO, F_GETFL, 0) | O_NONBLOCK);  // make stdout non blocking
    }
}

bool KeyboardController::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(KC) << "Config:" << rf.toString();

    auto usingRemoteRobot = rf.check("remoteRobot", "remote robot port");
    auto usingRemoteCartesian = rf.check("remoteCartesian", "remote cartesian port");

    if (!usingRemoteRobot && !usingRemoteCartesian)
    {
        yCError(KC) << "You cannot skip both controllers";
        return false;
    }

    auto localPrefix = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PREFIX), "local port prefix").asString();

    if (usingRemoteRobot)
    {
        yarp::os::Property controlBoardClientOptions {
            {"device", yarp::os::Value("remote_controlboard")},
            {"local", yarp::os::Value(localPrefix + "/joint")},
            {"remote", yarp::os::Value(rf.find("remoteRobot"))}
        };

        if (!controlBoardDevice.open(controlBoardClientOptions))
        {
            yCError(KC) << "Unable to open control board client device";
            return false;
        }

        if (!controlBoardDevice.view(iEncoders))
        {
            yCError(KC) << "Could not view iEncoders";
            return false;
        }

        if (!controlBoardDevice.view(iControlMode))
        {
            yCError(KC) << "Could not view iControlMode";
            return false;
        }

        if (!controlBoardDevice.view(iControlLimits))
        {
            yCError(KC) << "Could not view iControlLimits";
            return false;
        }

        if (!controlBoardDevice.view(iPositionControl))
        {
            yCError(KC) << "Could not view iPositionControl";
            return false;
        }

        if (!controlBoardDevice.view(iVelocityControl))
        {
            yCError(KC) << "Could not view iVelocityControl";
            return false;
        }

        if (!iEncoders->getAxes(&axes) || axes > MAX_JOINTS)
        {
            yCError(KC, "Number of joints (%d) exceeds supported limit (%d)", axes, MAX_JOINTS);
            return false;
        }

        minPositionLimits.resize(axes);
        maxPositionLimits.resize(axes);
        maxVelocityLimits.resize(axes);

        for (int i = 0; i < axes; i++)
        {
            double minPos, maxPos;

            if (!iControlLimits->getLimits(i, &minPos, &maxPos))
            {
                yCError(KC) << "Unable to retrieve joint limits";
                return false;
            }

            minPositionLimits[i] = minPos;
            maxPositionLimits[i] = maxPos;

            double minVel, maxVel;

            if (!iControlLimits->getVelLimits(i, &minVel, &maxVel))
            {
                yCError(KC) << "Unable to retrieve joint velocity limits";
                return false;
            }

            maxVelocityLimits[i] = maxVel;
        }

        currentJointVels.resize(axes, 0.0);

        auto jointModeStr = rf.check("jointMode", yarp::os::Value(DEFAULT_JOINT_MODE), "joint mode ('pos': position, 'vel': velocity)").asString();

        if (jointModeStr == "pos")
        {
            jointMode = POSITION;
        }
        else if (jointModeStr == "vel")
        {
            jointMode = VELOCITY;
        }
        else
        {
            yCError(KC) << "Unrecognized \"jointMode\" option:" << jointModeStr << "(available: 'pos', 'vel')";
            return false;
        }

        jointPosStep = rf.check("jointPosStep", yarp::os::Value(DEFAULT_JOINT_POSITION_STEP), "joint position step [deg]").asFloat32();
        jointVelStep = rf.check("jointVelStep", yarp::os::Value(DEFAULT_JOINT_VELOCITY_STEP), "joint velocity step [deg/s]").asFloat32();
    }

    if (usingRemoteCartesian)
    {
        yarp::os::Property cartesianControlClientOptions {
            {"device", yarp::os::Value("CartesianControlClient")},
            {"cartesianLocal", yarp::os::Value(localPrefix + "/cartesian")},
            {"cartesianRemote", yarp::os::Value(rf.find("remoteCartesian"))},
        };

        if (!cartesianControlDevice.open(cartesianControlClientOptions))
        {
            yCError(KC) << "Unable to open cartesian control client device";
            return false;
        }

        if (!cartesianControlDevice.view(iCartesianControl))
        {
            yCError(KC) << "Could not view iCartesianControl";
            return false;
        }

        double frameDouble;

        if (!iCartesianControl->getParameter(VOCAB_CC_CONFIG_FRAME, &frameDouble))
        {
            yCError(KC) << "Could not retrieve current frame";
            return false;
        }

        if (frameDouble != ICartesianSolver::BASE_FRAME && frameDouble != ICartesianSolver::TCP_FRAME)
        {
            yCError(KC) << "Unrecognized or unsupported frame";
            return false;
        }

        cartFrame = static_cast<ICartesianSolver::reference_frame>(frameDouble);

        angleRepr = rf.check("angleRepr", yarp::os::Value(DEFAULT_ANGLE_REPR), "angle representation").asString();

        if (!KinRepresentation::parseEnumerator(angleRepr, &orient, KinRepresentation::orientation_system::AXIS_ANGLE))
        {
            yCWarning(KC, "Unable to parse \"angleRepr\" option (%s), defaulting to %s", angleRepr.c_str(), DEFAULT_ANGLE_REPR);
            angleRepr = DEFAULT_ANGLE_REPR;
        }

        currentCartVels.resize(NUM_CART_COORDS, 0.0);

        usingThread = rf.check("pose", "use POSE command");
        usingThread = usingThread || rf.check("movi", "use POSE command"); // deprecated

        // `moviPeriodMs` is deprecated
        int threadMs = rf.check("posePeriodMs", rf.check("moviPeriodMs", yarp::os::Value(DEFAULT_THREAD_MS)), "POSE thread period [ms]").asInt32();

        if (usingThread)
        {
            linTrajThread = new LinearTrajectoryThread(threadMs, iCartesianControl);

            if (!linTrajThread->checkStreamingConfig())
            {
                yCError(KC) << "Unable to check streaming configuration";
                return false;
            }

            linTrajThread->useTcpFrame(cartFrame == ICartesianSolver::TCP_FRAME);
            linTrajThread->suspend(); // start in suspended state

            if (!linTrajThread->start())
            {
                yCError(KC) << "Unable to start POSE thread";
                return false;
            }
        }
    }

    issueStop(); // just in case
    ttyset();
    printHelp();

    return true;
}

bool KeyboardController::updateModule()
{
    char key;

    if (!readKey(&key))
    {
        return true;
    }

    switch (key)
    {
    // force application exit
    case 27:  // return
        stopModule();  // issues stop command at interruptModule()
        break;
    // print help
    case '?':
        printHelp();
        break;
    // print current joint positions
    case 'j':
        printJointPositions();
        break;
    // print current cartesian positions
    case 'p':
        printCartesianPositions();
        break;
    // joint velocity commands
    case '1':
        incrementOrDecrementJointCommand(Q1, increment_functor);
        break;
    case 'q':
        incrementOrDecrementJointCommand(Q1, decrement_functor);
        break;
    case '2':
        incrementOrDecrementJointCommand(Q2, increment_functor);
        break;
    case 'w':
        incrementOrDecrementJointCommand(Q2, decrement_functor);
        break;
    case '3':
        incrementOrDecrementJointCommand(Q3, increment_functor);
        break;
    case 'e':
        incrementOrDecrementJointCommand(Q3, decrement_functor);
        break;
    case '4':
        incrementOrDecrementJointCommand(Q4, increment_functor);
        break;
    case 'r':
        incrementOrDecrementJointCommand(Q4, decrement_functor);
        break;
    case '5':
        incrementOrDecrementJointCommand(Q5, increment_functor);
        break;
    case 't':
        incrementOrDecrementJointCommand(Q5, decrement_functor);
        break;
    case '6':
        incrementOrDecrementJointCommand(Q6, increment_functor);
        break;
    case 'y':
        incrementOrDecrementJointCommand(Q6, decrement_functor);
        break;
    case '7':
        incrementOrDecrementJointCommand(Q7, increment_functor);
        break;
    case 'u':
        incrementOrDecrementJointCommand(Q7, decrement_functor);
        break;
    case '8':
        incrementOrDecrementJointCommand(Q8, increment_functor);
        break;
    case 'i':
        incrementOrDecrementJointCommand(Q8, decrement_functor);
        break;
    case '9':
        incrementOrDecrementJointCommand(Q9, increment_functor);
        break;
    case 'o':
        incrementOrDecrementJointCommand(Q9, decrement_functor);
        break;
    // cartesian velocity commands
    case 'a':
        incrementOrDecrementCartesianCommand(X, increment_functor);
        break;
    case 'z':
        incrementOrDecrementCartesianCommand(X, decrement_functor);
        break;
    case 's':
        incrementOrDecrementCartesianCommand(Y, increment_functor);
        break;
    case 'x':
        incrementOrDecrementCartesianCommand(Y, decrement_functor);
        break;
    case 'd':
        incrementOrDecrementCartesianCommand(Z, increment_functor);
        break;
    case 'c':
        incrementOrDecrementCartesianCommand(Z, decrement_functor);
        break;
    case 'f':
        incrementOrDecrementCartesianCommand(ROTX, increment_functor);
        break;
    case 'v':
        incrementOrDecrementCartesianCommand(ROTX, decrement_functor);
        break;
    case 'g':
        incrementOrDecrementCartesianCommand(ROTY, increment_functor);
        break;
    case 'b':
        incrementOrDecrementCartesianCommand(ROTY, decrement_functor);
        break;
    case 'h':
        incrementOrDecrementCartesianCommand(ROTZ, increment_functor);
        break;
    case 'n':
        incrementOrDecrementCartesianCommand(ROTZ, decrement_functor);
        break;
    case '0':
        toggleJointMode();
        break;
    // toggle reference frame for cartesian commands
    case 'm':
        toggleReferenceFrame();
        break;
    // actuate tool (open gripper)
    case 'k':
        actuateTool(VOCAB_CC_ACTUATOR_OPEN_GRIPPER);
        break;
    // actuate tool (close gripper)
    case 'l':
        actuateTool(VOCAB_CC_ACTUATOR_CLOSE_GRIPPER);
        break;
    // issue stop
    case 13:  // enter
    default:
        issueStop();
        break;
    }

    return true;
}

bool KeyboardController::interruptModule()
{
    issueStop();
    std::cout << "Exiting..." << std::endl;
    ttyreset(0);

    return true;
}

double KeyboardController::getPeriod()
{
    return 0.01; // [s]
}

bool KeyboardController::close()
{
    if (cartesianControlDevice.isValid() && usingThread)
    {
        linTrajThread->stop();
        delete linTrajThread;
        linTrajThread = nullptr;
    }

    controlBoardDevice.close();
    cartesianControlDevice.close();

    return true;
}

template <typename func>
void KeyboardController::incrementOrDecrementJointCommand(joint j, func op)
{
    if (!controlBoardDevice.isValid())
    {
        yCWarning(KC) << "Unrecognized command (you chose not to launch remote control board client)";
        issueStop();
        return;
    }

    if (controlMode != JOINT_MODE)
    {
        issueStop();
    }

    if (axes <= j)
    {
        yCWarning(KC) << "Unrecognized key, only" << axes << "joints available";
        issueStop();
        return;
    }

    if (jointMode == POSITION)
    {
        if (!iControlMode->setControlModes(std::vector(axes, VOCAB_CM_POSITION).data()))
        {
            yCError(KC) << "setPositionModes failed";
            issueStop();
            return;
        }

        std::vector<double> q(axes);

        if (!iEncoders->getEncoders(q.data()))
        {
            yCError(KC) << "getEncoders failed";
            issueStop();
            return;
        }

        q[j] = op(q[j], jointPosStep);

        if (q[j] < minPositionLimits[j] || q[j] > maxPositionLimits[j])
        {
            yCWarning(KC, "Joint position limit exceeded: minPos[%d] = %f, maxPos[%d] = %f", j, minPositionLimits[j], j, maxPositionLimits[j]);
            q[j] = std::clamp(q[j], minPositionLimits[j], maxPositionLimits[j]);
        }

        std::cout << "New joint position: " << roundZeroes(q) << std::endl;

        if (!iPositionControl->positionMove(j, q[j]))
        {
            yCError(KC) << "positionMove failed";
            issueStop();
            return;
        }

        controlMode = JOINT_MODE;
    }
    else if (jointMode == VELOCITY)
    {
        if (!iControlMode->setControlModes(std::vector(axes, VOCAB_CM_VELOCITY).data()))
        {
            yCError(KC) << "setVelocityModes failed";
            issueStop();
            return;
        }

        currentJointVels[j] = op(currentJointVels[j], jointVelStep);

        if (std::abs(currentJointVels[j]) > maxVelocityLimits[j])
        {
            yCWarning(KC, "Absolute joint velocity limit exceeded: maxVel[%d] = %f", j, maxVelocityLimits[j]);
            currentJointVels[j] = op(0, 1) * maxVelocityLimits[j];
        }

        std::cout << "New joint velocity: " << roundZeroes(currentJointVels) << std::endl;

        if (!iVelocityControl->velocityMove(j, currentJointVels[j]))
        {
            yCError(KC) << "velocityMove failed";
            issueStop();
            return;
        }

        controlMode = JOINT_MODE;
    }
    else
    {
        yCError(KC) << "Unrecognized joint mode";
        issueStop();
        return;
    }
}

template <typename func>
void KeyboardController::incrementOrDecrementCartesianCommand(cart coord, func op)
{
    if (!cartesianControlDevice.isValid())
    {
        yCWarning(KC) << "Unrecognized command (you chose not to launch cartesian controller client)";
        issueStop();
        return;
    }

    if (controlMode != CARTESIAN_MODE)
    {
        issueStop();
    }

    bool isLinear = coord == X || coord == Y || coord == Z;
    double step = isLinear ? CARTESIAN_LINEAR_VELOCITY_STEP : CARTESIAN_ANGULAR_VELOCITY_STEP;

    currentCartVels[coord] = op(currentCartVels[coord], step);

    std::cout << "New cartesian velocity: " << roundZeroes(currentCartVels) << std::endl;

    if (roundZeroes(currentCartVels) == ZERO_CARTESIAN_VELOCITY)
    {
        currentCartVels = ZERO_CARTESIAN_VELOCITY; // send vector of zeroes

        if (usingThread)
        {
            linTrajThread->suspend();
        }
        else
        {
            iCartesianControl->movv(currentCartVels);
        }
    }
    else
    {
        if (usingThread)
        {
            if (!linTrajThread->configure(currentCartVels))
            {
                yCError(KC) << "Unable to configure cartesian command";
                return;
            }

            linTrajThread->resume();
        }
        else
        {
            iCartesianControl->movv(currentCartVels);
        }
    }

    controlMode = CARTESIAN_MODE;
}

void KeyboardController::toggleJointMode()
{
    if (!controlBoardDevice.isValid())
    {
        yCWarning(KC) << "Unrecognized command (you chose not to launch remote control board client)";
        issueStop();
        return;
    }

    issueStop();

    if (jointMode == POSITION)
    {
        jointMode = VELOCITY;
    }
    else if (jointMode == VELOCITY)
    {
        jointMode = POSITION;
    }

    std::cout << "Toggled joint control mode: " << (jointMode == POSITION ? "position" : "velocity");
    std::cout << " (step: " << (jointMode == POSITION ? jointPosStep : jointVelStep);
    std::cout << " " << (jointMode == POSITION ? "[degrees]" : "[degrees/s]") << ")" << std::endl;
}

void KeyboardController::toggleReferenceFrame()
{
    if (!cartesianControlDevice.isValid())
    {
        yCWarning(KC) << "Unrecognized command (you chose not to launch cartesian controller client)";
        issueStop();
        return;
    }

    issueStop();

    ICartesianSolver::reference_frame newFrame;
    std::string str;

    switch (cartFrame)
    {
    case ICartesianSolver::BASE_FRAME:
        newFrame = ICartesianSolver::TCP_FRAME;
        str = "end effector";
        break;
    case ICartesianSolver::TCP_FRAME:
        newFrame = ICartesianSolver::BASE_FRAME;
        str = "inertial";
        break;
    default:
        str = "unknown";
        break;
    }

    if (str != "unknown")
    {
        if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, newFrame))
        {
            yCError(KC) << "Unable to set reference frame:" << str;
            return;
        }

        cartFrame = newFrame;
    }

    if (usingThread)
    {
        linTrajThread->useTcpFrame(newFrame == ICartesianSolver::TCP_FRAME);
    }

    std::cout << "Toggled reference frame for cartesian commands: " << str << std::endl;
}

void KeyboardController::actuateTool(int command)
{
    if (!cartesianControlDevice.isValid())
    {
        yCWarning(KC) << "Unrecognized command (you chose not to launch cartesian controller client)";
        issueStop();
        return;
    }

    if (!iCartesianControl->act(command))
    {
        yCError(KC) << "Unable to send" << yarp::os::Vocab32::decode(command) << "command to actuator";
    }
    else
    {
        currentActuatorCommand = command;
    }
}

void KeyboardController::printJointPositions()
{
    if (!controlBoardDevice.isValid())
    {
        yCWarning(KC) << "Unrecognized command (you chose not to launch remote control board client)";
        issueStop();
        return;
    }

    std::vector<double> encs(axes);
    iEncoders->getEncoders(encs.data());

    std::cout << "Current joint positions [degrees]:" << std::endl;
    std::cout << roundZeroes(encs) << std::endl;
}

void KeyboardController::printCartesianPositions()
{
    if (!cartesianControlDevice.isValid())
    {
        yCWarning(KC) << "Unrecognized command (you chose not to launch cartesian controller client)";
        issueStop();
        return;
    }

    std::vector<double> x;
    iCartesianControl->stat(x);
    KinRepresentation::decodePose(x, x, KinRepresentation::coordinate_system::CARTESIAN, orient, KinRepresentation::angular_units::DEGREES);

    std::cout << "Current cartesian positions [meters, degrees (" << angleRepr << ")]: " << std::endl;
    std::cout << roundZeroes(x) << std::endl;
}

void KeyboardController::issueStop()
{
    if (cartesianControlDevice.isValid())
    {
        if (currentActuatorCommand != VOCAB_CC_ACTUATOR_NONE)
        {
            if (!iCartesianControl->act(VOCAB_CC_ACTUATOR_STOP_GRIPPER))
            {
                yCWarning(KC) << "Unable to stop actuator";
            }
            else
            {
                currentActuatorCommand = VOCAB_CC_ACTUATOR_NONE;
            }
        }

        if (usingThread)
        {
            linTrajThread->suspend();
        }

        if (!iCartesianControl->stopControl())
        {
            yCWarning(KC) << "Unable to stop cartesian control";
        }
    }
    else if (controlBoardDevice.isValid())
    {
        if (jointMode == POSITION && !iPositionControl->stop())
        {
            yCWarning(KC) << "Unable to stop position control";
        }
        else if (jointMode == VELOCITY && !iVelocityControl->stop())
        {
            yCWarning(KC) << "Unable to stop velocity control";
        }
    }

    if (controlBoardDevice.isValid())
    {
        std::fill(currentJointVels.begin(), currentJointVels.end(), 0.0);
    }

    if (cartesianControlDevice.isValid())
    {
        std::fill(currentCartVels.begin(), currentCartVels.end(), 0.0);
    }

    std::cout << "Stopped" << std::endl;

    controlMode = NOT_CONTROLLING;
}

void KeyboardController::printHelp()
{
    static const int markerWidth = 70;

    std::cout << std::string(markerWidth, '-') << std::endl;
    std::cout << " [Esc] - close the application" << std::endl;
    std::cout << " '?' - print this help guide" << std::endl;

    if (controlBoardDevice.isValid())
    {
        std::cout << " 'j' - print current joint positions" << std::endl;
    }

    if (cartesianControlDevice.isValid())
    {
        std::cout << " 'p' - print current cartesian positions (angleRepr: " << angleRepr << ")" << std::endl;
    }

    if (controlBoardDevice.isValid())
    {
        static const char jointPos[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        static const char jointNeg[] = {'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o'};

        std::cout << " '" << jointPos[0] << "'";

        if (axes > 1)
        {
            std::cout << " to '" << jointPos[axes - 1] << "', ";
        }
        else
        {
            std::cout << "/";
        }

        std::cout << "'" << jointNeg[0] << "'";

        if (axes > 1)
        {
            std::cout << " to '" << jointNeg[axes - 1] << "'";
        }

        std::cout << " - issue joint movements (+/-)" << std::endl;

        std::cout << " '0' - toggle joint mode (current: ";

        if (jointMode == POSITION)
        {
            std::cout << "position, step: " << jointPosStep << " [degrees])";
        }
        else
        {
            std::cout << "velocity, step: " << jointVelStep << " [degrees/s])";
        }

        std::cout << std::endl;
    }

    if (cartesianControlDevice.isValid())
    {
        std::cout << " 'a'/'z' - move along x axis (+/-)" << std::endl;
        std::cout << " 's'/'x' - move along y axis (+/-)" << std::endl;
        std::cout << " 'd'/'c' - move along z axis (+/-)" << std::endl;
        std::cout << " 'f'/'v' - rotate about x axis (+/-)" << std::endl;
        std::cout << " 'g'/'b' - rotate about y axis (+/-)" << std::endl;
        std::cout << " 'h'/'n' - rotate about z axis (+/-)" << std::endl;

        std::cout << " 'm' - toggle reference frame (current: ";
        std::cout << (cartFrame == ICartesianSolver::BASE_FRAME ? "inertial" : "end effector") << ")" << std::endl;

        std::cout << " 'k'/'l' - open/close gripper" << std::endl;
    }

    std::cout << " [Enter] - issue stop" << std::endl;
    std::cout << std::string(markerWidth, '-') << std::endl;
}

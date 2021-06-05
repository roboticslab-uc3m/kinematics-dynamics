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

using namespace roboticslab;

namespace
{
    struct termios ots;

    bool readKey(char * key)
    {
        return read(STDIN_FILENO, key, 1) > 0;
    }

    // https://stackoverflow.com/a/23397700
    std::ostream& operator<<(std::ostream& out, const std::vector<double>& v)
    {
        if (!v.empty())
        {
            out << '[';
            std::copy(v.begin(), v.end(), std::ostream_iterator<double>(out, ", "));
            out << "\b\b]";
        }

        return out;
    }

    std::vector<double> roundZeroes(const std::vector<double>& v_in)
    {
        static const double precision = 1e-6;

        std::vector<double> v_out(v_in);

        for (std::vector<double>::iterator it = v_out.begin(); it != v_out.end(); ++it)
        {
            if (std::abs(*it) < precision)
            {
                *it = 0.0;
            }
        }

        return v_out;
    }

    // reset the TTY configurations that was changed in the ttyset function (UNIX)
    void ttyreset(int signal)
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

const std::vector<double> ZERO_CARTESIAN_VELOCITY(KeyboardController::NUM_CART_COORDS);

constexpr auto JOINT_VELOCITY_STEP = 0.5; // [deg]
constexpr auto CARTESIAN_LINEAR_VELOCITY_STEP = 0.005; // [m]
constexpr auto CARTESIAN_ANGULAR_VELOCITY_STEP = 0.01; // [deg]

bool KeyboardController::configure(yarp::os::ResourceFinder & rf)
{
    yDebug() << "KeyboardController config:" << rf.toString();

    bool skipControlboardController = rf.check("skipRCB", "don't load remote control board client");
    bool skipCartesianController = rf.check("skipCC", "don't load cartesian control client");

    if (skipControlboardController && skipCartesianController)
    {
        yError() << "You cannot skip both controllers";
        return false;
    }

    if (!skipControlboardController)
    {
        std::string localRobot = rf.check("localRobot", yarp::os::Value(DEFAULT_ROBOT_LOCAL),
                "local robot port").asString();
        std::string remoteRobot = rf.check("remoteRobot", yarp::os::Value(DEFAULT_ROBOT_REMOTE),
                "remote robot port").asString();

        yarp::os::Property controlboardClientOptions {
            {"device", yarp::os::Value("remote_controlboard")},
            {"local", yarp::os::Value(localRobot)},
            {"remote", yarp::os::Value(remoteRobot)}
        };

        if (!controlboardDevice.open(controlboardClientOptions))
        {
            yError() << "Unable to open control board client device";
            return false;
        }

        if (!controlboardDevice.view(iEncoders))
        {
            yError() << "Could not view iEncoders";
            return false;
        }

        if (!controlboardDevice.view(iControlMode))
        {
            yError() << "Could not view iControlMode";
            return false;
        }

        if (!controlboardDevice.view(iControlLimits))
        {
            yError() << "Could not view iControlLimits";
            return false;
        }

        if (!controlboardDevice.view(iVelocityControl))
        {
            yError() << "Could not view iVelocityControl";
            return false;
        }

        iEncoders->getAxes(&axes);

        if (axes > MAX_JOINTS)
        {
            yError("Number of joints (%d) exceeds supported limit (%d)", axes, MAX_JOINTS);
            return false;
        }

        maxVelocityLimits.resize(axes);

        for (int i = 0; i < axes; i++)
        {
            double min, max;
            iControlLimits->getVelLimits(i, &min, &max);
            maxVelocityLimits[i] = max;
        }

        currentJointVels.resize(axes, 0.0);
    }

    if (!skipCartesianController)
    {
        std::string localCartesian = rf.check("localCartesian", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),
                "local cartesian port").asString();
        std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
                "remote cartesian port").asString();

        yarp::os::Property cartesianControlClientOptions {
            {"device", yarp::os::Value("CartesianControlClient")},
            {"cartesianLocal", yarp::os::Value(localCartesian)},
            {"cartesianRemote", yarp::os::Value(remoteCartesian)},
        };

        if (!cartesianControlDevice.open(cartesianControlClientOptions))
        {
            yError() << "Unable to open cartesian control client device";
            return false;
        }

        if (!cartesianControlDevice.view(iCartesianControl))
        {
            yError() << "Could not view iCartesianControl";
            return false;
        }

        double frameDouble;

        if (!iCartesianControl->getParameter(VOCAB_CC_CONFIG_FRAME, &frameDouble))
        {
            yError() << "Could not retrieve current frame";
            return false;
        }

        if (frameDouble != ICartesianSolver::BASE_FRAME && frameDouble != ICartesianSolver::TCP_FRAME)
        {
            yError() << "Unrecognized or unsupported frame";
            return false;
        }

        cartFrame = static_cast<ICartesianSolver::reference_frame>(frameDouble);

        angleRepr = rf.check("angleRepr", yarp::os::Value(DEFAULT_ANGLE_REPR), "angle representation").asString();

        if (!KinRepresentation::parseEnumerator(angleRepr, &orient, KinRepresentation::orientation_system::AXIS_ANGLE))
        {
            yWarning("Unable to parse \"angleRepr\" option (%s), defaulting to %s", angleRepr.c_str(), DEFAULT_ANGLE_REPR);
            angleRepr = DEFAULT_ANGLE_REPR;
        }

        currentCartVels.resize(NUM_CART_COORDS, 0.0);

        usingThread = rf.check("movi", "use MOVI command");

        int threadMs = rf.check("moviPeriodMs", yarp::os::Value(DEFAULT_THREAD_MS), "MOVI thread period [ms]").asInt32();

        if (usingThread)
        {
            linTrajThread = new LinearTrajectoryThread(threadMs, iCartesianControl);

            if (!linTrajThread->checkStreamingConfig())
            {
                yError() << "Unable to check streaming configuration";
                return false;
            }

            linTrajThread->useTcpFrame(cartFrame == ICartesianSolver::TCP_FRAME);
            linTrajThread->suspend(); // start in suspended state

            if (!linTrajThread->start())
            {
                yError() << "Unable to start MOVI thread";
                return false;
            }
        }
    }

    currentActuatorCommand = VOCAB_CC_ACTUATOR_NONE;

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
        incrementOrDecrementJointVelocity(Q1, increment_functor);
        break;
    case 'q':
        incrementOrDecrementJointVelocity(Q1, decrement_functor);
        break;
    case '2':
        incrementOrDecrementJointVelocity(Q2, increment_functor);
        break;
    case 'w':
        incrementOrDecrementJointVelocity(Q2, decrement_functor);
        break;
    case '3':
        incrementOrDecrementJointVelocity(Q3, increment_functor);
        break;
    case 'e':
        incrementOrDecrementJointVelocity(Q3, decrement_functor);
        break;
    case '4':
        incrementOrDecrementJointVelocity(Q4, increment_functor);
        break;
    case 'r':
        incrementOrDecrementJointVelocity(Q4, decrement_functor);
        break;
    case '5':
        incrementOrDecrementJointVelocity(Q5, increment_functor);
        break;
    case 't':
        incrementOrDecrementJointVelocity(Q5, decrement_functor);
        break;
    case '6':
        incrementOrDecrementJointVelocity(Q6, increment_functor);
        break;
    case 'y':
        incrementOrDecrementJointVelocity(Q6, decrement_functor);
        break;
    case '7':
        incrementOrDecrementJointVelocity(Q7, increment_functor);
        break;
    case 'u':
        incrementOrDecrementJointVelocity(Q7, decrement_functor);
        break;
    case '8':
        incrementOrDecrementJointVelocity(Q8, increment_functor);
        break;
    case 'i':
        incrementOrDecrementJointVelocity(Q8, decrement_functor);
        break;
    case '9':
        incrementOrDecrementJointVelocity(Q9, increment_functor);
        break;
    case 'o':
        incrementOrDecrementJointVelocity(Q9, decrement_functor);
        break;
    // cartesian velocity commands
    case 'a':
        incrementOrDecrementCartesianVelocity(X, increment_functor);
        break;
    case 'z':
        incrementOrDecrementCartesianVelocity(X, decrement_functor);
        break;
    case 's':
        incrementOrDecrementCartesianVelocity(Y, increment_functor);
        break;
    case 'x':
        incrementOrDecrementCartesianVelocity(Y, decrement_functor);
        break;
    case 'd':
        incrementOrDecrementCartesianVelocity(Z, increment_functor);
        break;
    case 'c':
        incrementOrDecrementCartesianVelocity(Z, decrement_functor);
        break;
    case 'f':
        incrementOrDecrementCartesianVelocity(ROTX, increment_functor);
        break;
    case 'v':
        incrementOrDecrementCartesianVelocity(ROTX, decrement_functor);
        break;
    case 'g':
        incrementOrDecrementCartesianVelocity(ROTY, increment_functor);
        break;
    case 'b':
        incrementOrDecrementCartesianVelocity(ROTY, decrement_functor);
        break;
    case 'h':
        incrementOrDecrementCartesianVelocity(ROTZ, increment_functor);
        break;
    case 'n':
        incrementOrDecrementCartesianVelocity(ROTZ, decrement_functor);
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

    controlboardDevice.close();
    cartesianControlDevice.close();

    return true;
}

template <typename func>
void KeyboardController::incrementOrDecrementJointVelocity(joint q, func op)
{
    if (!controlboardDevice.isValid())
    {
        yWarning() << "Unrecognized command (you chose not to launch remote control board client)";
        issueStop();
        return;
    }

    if (controlMode == CARTESIAN_MODE)
    {
        issueStop();
    }

    if (axes <= q)
    {
        yWarning() << "Unrecognized key, only" << axes << "joints available";
        issueStop();
        return;
    }

    std::vector<int> velModes(axes, VOCAB_CM_VELOCITY);

    if (!iControlMode->setControlModes(velModes.data()))
    {
        yError() << "setVelocityModes failed";
        issueStop();
        return;
    }

    currentJointVels[q] = op(currentJointVels[q], JOINT_VELOCITY_STEP);

    if (std::abs(currentJointVels[q]) > maxVelocityLimits[q])
    {
        yWarning("Absolute joint velocity limit exceeded: maxVel[%d] = %f", q, maxVelocityLimits[q]);
        currentJointVels[q] = op(0, 1) * maxVelocityLimits[q];
    }

    std::cout << "New joint velocity: " << roundZeroes(currentJointVels) << std::endl;

    if (!iVelocityControl->velocityMove(q, currentJointVels[q]))
    {
        yError() << "velocityMove failed";
        issueStop();
    }
    else
    {
        controlMode = JOINT_MODE;
    }
}

template <typename func>
void KeyboardController::incrementOrDecrementCartesianVelocity(cart coord, func op)
{
    if (!cartesianControlDevice.isValid())
    {
        yWarning() << "Unrecognized command (you chose not to launch cartesian controller client)";
        issueStop();
        return;
    }

    if (controlMode == JOINT_MODE)
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
            iCartesianControl->twist(currentCartVels); // disable CMC
        }
    }
    else
    {
        if (usingThread)
        {
            if (!linTrajThread->configure(currentCartVels))
            {
                yError() << "Unable to configure cartesian command";
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

void KeyboardController::toggleReferenceFrame()
{
    if (!cartesianControlDevice.isValid())
    {
        yWarning() << "Unrecognized command (you chose not to launch cartesian controller client)";
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
            yError() << "Unable to set reference frame:" << str;
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
        yWarning() << "Unrecognized command (you chose not to launch cartesian controller client)";
        issueStop();
        return;
    }

    if (!iCartesianControl->act(command))
    {
        yError() << "Unable to send" << yarp::os::Vocab::decode(command) << "command to actuator";
    }
    else
    {
        currentActuatorCommand = command;
    }
}

void KeyboardController::printJointPositions()
{
    if (!controlboardDevice.isValid())
    {
        yWarning() << "Unrecognized command (you chose not to launch remote control board client)";
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
        yWarning() << "Unrecognized command (you chose not to launch cartesian controller client)";
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
                yWarning() << "Unable to stop actuator";
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

        iCartesianControl->stopControl();
    }
    else if (controlboardDevice.isValid())
    {
        iVelocityControl->stop();
    }

    if (controlboardDevice.isValid())
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

    if (controlboardDevice.isValid())
    {
        std::cout << " 'j' - query current joint positions" << std::endl;
    }

    if (cartesianControlDevice.isValid())
    {
        std::cout << " 'p' - query current cartesian positions (angleRepr: " << angleRepr << ")" << std::endl;
    }

    if (controlboardDevice.isValid())
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

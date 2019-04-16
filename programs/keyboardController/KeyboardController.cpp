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

#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

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

const std::vector<double> roboticslab::KeyboardController::ZERO_CARTESIAN_VELOCITY(NUM_CART_COORDS);

const double roboticslab::KeyboardController::JOINT_VELOCITY_STEP = 0.5;  // [deg]
const double roboticslab::KeyboardController::CARTESIAN_LINEAR_VELOCITY_STEP = 0.005;  // [m]
const double roboticslab::KeyboardController::CARTESIAN_ANGULAR_VELOCITY_STEP = 0.01;  // [deg]

bool roboticslab::KeyboardController::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("KeyboardController config: %s.\n", rf.toString().c_str());

    bool skipControlboardController = rf.check("skipRCB", "don't load remote control board client");
    bool skipCartesianController = rf.check("skipCC", "don't load cartesian control client");

    if (skipControlboardController && skipCartesianController)
    {
        CD_ERROR("You cannot skip both controllers.\n");
        return false;
    }

    if (!skipControlboardController)
    {
        std::string localRobot = rf.check("localRobot", yarp::os::Value(DEFAULT_ROBOT_LOCAL),
                "local robot port").asString();
        std::string remoteRobot = rf.check("remoteRobot", yarp::os::Value(DEFAULT_ROBOT_REMOTE),
                "remote robot port").asString();

        yarp::os::Property controlboardClientOptions;
        controlboardClientOptions.put("device", "remote_controlboard");
        controlboardClientOptions.put("local", localRobot);
        controlboardClientOptions.put("remote", remoteRobot);

        controlboardDevice.open(controlboardClientOptions);

        if (!controlboardDevice.isValid())
        {
            CD_ERROR("controlboard client device not valid.\n");
            return false;
        }

        if (!controlboardDevice.view(iEncoders))
        {
            CD_ERROR("Could not view iEncoders.\n");
            return false;
        }

        if (!controlboardDevice.view(iControlMode))
        {
            CD_ERROR("Could not view iControlMode.\n");
            return false;
        }

        if (!controlboardDevice.view(iControlLimits))
        {
            CD_ERROR("Could not view iControlLimits.\n");
            return false;
        }

        if (!controlboardDevice.view(iVelocityControl))
        {
            CD_ERROR("Could not view iVelocityControl.\n");
            return false;
        }

        iEncoders->getAxes(&axes);

        if (axes > MAX_JOINTS)
        {
            CD_ERROR("Number of joints (%d) exceeds supported limit (%d).\n", axes, MAX_JOINTS);
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

        yarp::os::Property cartesianControlClientOptions;
        cartesianControlClientOptions.put("device", "CartesianControlClient");
        cartesianControlClientOptions.put("cartesianLocal", localCartesian);
        cartesianControlClientOptions.put("cartesianRemote", remoteCartesian);

        cartesianControlDevice.open(cartesianControlClientOptions);

        if (!cartesianControlDevice.isValid())
        {
            CD_ERROR("cartesian control client device not valid.\n");
            return false;
        }

        if (!cartesianControlDevice.view(iCartesianControl))
        {
            CD_ERROR("Could not view iCartesianControl.\n");
            return false;
        }

        double frameDouble;

        if (!iCartesianControl->getParameter(VOCAB_CC_CONFIG_FRAME, &frameDouble))
        {
            CD_ERROR("Could not retrieve current frame.\n");
            return false;
        }

        if (frameDouble != ICartesianSolver::BASE_FRAME && frameDouble != ICartesianSolver::TCP_FRAME)
        {
            CD_ERROR("Unrecognized or unsupported frame.\n");
            return false;
        }

        cartFrame = static_cast<ICartesianSolver::reference_frame>(frameDouble);

        angleRepr = rf.check("angleRepr", yarp::os::Value(DEFAULT_ANGLE_REPR), "angle representation").asString();

        if (!KinRepresentation::parseEnumerator(angleRepr, &orient, KinRepresentation::AXIS_ANGLE))
        {
            CD_WARNING("Unable to parse \"angleRepr\" option (%s), defaulting to %s.\n", angleRepr.c_str(), DEFAULT_ANGLE_REPR);
            angleRepr = DEFAULT_ANGLE_REPR;
        }

        currentCartVels.resize(NUM_CART_COORDS, 0.0);

        usingThread = rf.check("movi", "use MOVI command");

        int threadMs = rf.check("moviPeriodMs", yarp::os::Value(DEFAULT_THREAD_MS), "MOVI thread period [ms]").asInt();

        if (usingThread)
        {
            linTrajThread = new LinearTrajectoryThread(threadMs, iCartesianControl);

            if (!linTrajThread->checkStreamingConfig())
            {
                CD_ERROR("Unable to check streaming configuration.\n");
                return false;
            }

            linTrajThread->useTcpFrame(cartFrame == ICartesianSolver::TCP_FRAME);
            linTrajThread->suspend(); // start in suspended state

            if (!linTrajThread->start())
            {
                CD_ERROR("Unable to start MOVI thread.\n");
                return false;
            }
        }
    }

    issueStop(); // just in case

    ttyset();

    printHelp();

    controlMode = NOT_CONTROLLING;

    return true;
}

bool roboticslab::KeyboardController::updateModule()
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
    // issue stop
    case 13:  // enter
    default:
        issueStop();
        break;
    }

    return true;
}

bool roboticslab::KeyboardController::interruptModule()
{
    issueStop();
    std::cout << "Exiting..." << std::endl;
    ttyreset(0);

    return true;
}

double roboticslab::KeyboardController::getPeriod()
{
    return 0.01;  // [s]
}

bool roboticslab::KeyboardController::close()
{
    if (cartesianControlDevice.isValid() && usingThread)
    {
        linTrajThread->stop();
        delete linTrajThread;
        linTrajThread = 0;
    }

    controlboardDevice.close();
    cartesianControlDevice.close();

    return true;
}

template <typename func>
void roboticslab::KeyboardController::incrementOrDecrementJointVelocity(joint q, func op)
{
    if (!controlboardDevice.isValid())
    {
        CD_WARNING("Unrecognized command (you chose not to launch remote control board client).\n");
        issueStop();
        return;
    }

    if (controlMode == CARTESIAN_MODE)
    {
        issueStop();
    }

    if (axes <= q)
    {
        CD_WARNING("Unrecognized key, only %d joints available.\n", axes);
        issueStop();
        return;
    }

    std::vector<int> velModes(axes, VOCAB_CM_VELOCITY);

    if (!iControlMode->setControlModes(velModes.data()))
    {
        CD_ERROR("setVelocityModes failed\n");
        issueStop();
        return;
    }

    currentJointVels[q] = op(currentJointVels[q], JOINT_VELOCITY_STEP);

    if (std::abs(currentJointVels[q]) > maxVelocityLimits[q])
    {
        CD_WARNING("Absolute joint velocity limit exceeded: maxVel[%d] = %f\n", q, maxVelocityLimits[q]);
        currentJointVels[q] = op(0, 1) * maxVelocityLimits[q];
    }

    std::cout << "New joint velocity: " << roundZeroes(currentJointVels) << std::endl;

    if (!iVelocityControl->velocityMove(q, currentJointVels[q]))
    {
        CD_ERROR("velocityMove failed\n");
        issueStop();
    }
    else
    {
        controlMode = JOINT_MODE;
    }
}

template <typename func>
void roboticslab::KeyboardController::incrementOrDecrementCartesianVelocity(cart coord, func op)
{
    if (!cartesianControlDevice.isValid())
    {
        CD_WARNING("Unrecognized command (you chose not to launch cartesian controller client).\n");
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
                CD_ERROR("Unable to configure cartesian command.\n");
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

void roboticslab::KeyboardController::toggleReferenceFrame()
{
    if (!cartesianControlDevice.isValid())
    {
        CD_WARNING("Unrecognized command (you chose not to launch cartesian controller client).\n");
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
            CD_ERROR("Unable to set reference frame: %s.\n", str.c_str());
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

void roboticslab::KeyboardController::printJointPositions()
{
    if (!controlboardDevice.isValid())
    {
        CD_WARNING("Unrecognized command (you chose not to launch remote control board client).\n");
        issueStop();
        return;
    }

    std::vector<double> encs(axes);
    iEncoders->getEncoders(encs.data());

    std::cout << "Current joint positions [degrees]:" << std::endl;
    std::cout << roundZeroes(encs) << std::endl;
}

void roboticslab::KeyboardController::printCartesianPositions()
{
    if (!cartesianControlDevice.isValid())
    {
        CD_WARNING("Unrecognized command (you chose not to launch cartesian controller client).\n");
        issueStop();
        return;
    }

    std::vector<double> x;
    iCartesianControl->stat(x);
    KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, orient, KinRepresentation::DEGREES);

    std::cout << "Current cartesian positions [meters, degrees (" << angleRepr << ")]: " << std::endl;
    std::cout << roundZeroes(x) << std::endl;
}

void roboticslab::KeyboardController::issueStop()
{
    if (cartesianControlDevice.isValid())
    {
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

void roboticslab::KeyboardController::printHelp()
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
    }

    std::cout << " [Enter] - issue stop" << std::endl;
    std::cout << std::string(markerWidth, '-') << std::endl;
}

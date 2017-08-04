#include "RateControllerConsole.hpp"

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

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <ColorDebug.hpp>

namespace
{
    struct termios ots;

    bool readKey(char * key)
    {
        return read(STDIN_FILENO, key, 1) > 0;
    }

    // https://stackoverflow.com/a/23397700
    template <typename T>
    std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
    {
        if (!v.empty())
        {
            out << '[';
            std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
            out << "\b\b]";
        }

        return out;
    }

    // reset the TTY configurations that was changed in the ttyset function (UNIX)
    void ttyreset(int signal)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &ots);
        tcsetattr(STDOUT_FILENO, TCSANOW, &ots);
        std::exit(signal);
    }

    // configure the TTY for reading keyboard input (UNIX)
    void ttyset(void)
    {
        struct termios ts;
        struct sigaction sact;
        tcgetattr(STDIN_FILENO, &ts);
        ots = ts;
        ts.c_lflag &= ~ICANON;  // raw data mode
        ts.c_lflag &= ~(ECHO | ECHOCTL | ECHONL);  // no echo
        ts.c_lflag |= IEXTEN;

        // restore tty after these signals
        sact.sa_handler = ttyreset;
        sigaction(SIGHUP, &sact, NULL);
        sigaction(SIGINT, &sact, NULL);
        sigaction(SIGPIPE, &sact, NULL);
        sigaction(SIGTERM, &sact, NULL);
        tcsetattr(STDIN_FILENO, TCSANOW, &ts);  // set raw data mode
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);  // make stdin non blocking
        fcntl(STDOUT_FILENO, F_SETFL, fcntl(STDOUT_FILENO, F_GETFL, 0) | O_NONBLOCK);  // make stdout non blocking
    }
}

const double roboticslab::RateControllerConsole::JOINT_VELOCITY_STEP = 0.5;  // [deg]
const double roboticslab::RateControllerConsole::CARTESIAN_LINEAR_VELOCITY_STEP = 0.005;  // [m]
const double roboticslab::RateControllerConsole::CARTESIAN_ANGULAR_VELOCITY_STEP = 0.01;  // [deg]

bool roboticslab::RateControllerConsole::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("RateControllerConsole config: %s.\n", rf.toString().c_str());

    std::string localRobot = rf.check("localRobot", yarp::os::Value(DEFAULT_ROBOT_LOCAL), "local robot port").asString();
    std::string remoteRobot = rf.check("remoteRobot", yarp::os::Value(DEFAULT_ROBOT_REMOTE), "remote robot port").asString();

    std::string localCartesian = rf.check("localCartesian", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL), "local cartesian port").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE), "remote cartesian port").asString();

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

    iEncoders->getAxes(&axes);

    if (axes > MAX_JOINTS)
    {
        CD_ERROR("Number of joints (%d) exceeds supported limit (%d).\n", axes, MAX_JOINTS);
        cartesianControlDevice.close();
        controlboardDevice.close();
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
    currentCartVels.resize(NUM_CART_COORDS, 0.0);

    ttyset();

    return true;
}

bool roboticslab::RateControllerConsole::updateModule()
{
    char key;

    if (readKey(&key))
    {
        switch (key)
        {
        // force application exit
        case 27:  // return
            {
                iCartesianControl->stopControl();
                std::cout << "Exiting..." << std::endl;
                stopModule();
            }
            break;
        // print help
        case '?':
            {
                std::cout << "RTFM" << std::endl;
            }
            break;
        // print current joint positions
        case 'j':
            {
                std::vector<double> encs(axes);
                iEncoders->getEncoders(encs.data());
                std::cout << "Current joint positions: " << encs << std::endl;
            }
            break;
        // print current cartesian positions
        case 'p':
            {
                int state;
                std::vector<double> x;
                iCartesianControl->stat(state, x);
                std::cout << "Current cartesian positions: " << x << std::endl;
            }
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
        // issue stop
        case 13:  // enter
        default:
            issueStop();
            break;
        }
    }

    return true;
}

bool roboticslab::RateControllerConsole::interruptModule()
{
    bool ok = true;
    ok &= iCartesianControl->stopControl();
    ok &= cartesianControlDevice.close();
    ok &= controlboardDevice.close();
    return ok;
}

double roboticslab::RateControllerConsole::getPeriod()
{
    return 0.01;  // [s]
}

template <typename func>
void roboticslab::RateControllerConsole::incrementOrDecrementJointVelocity(joint q, func op)
{
    if (axes > q)
    {
        for (int i = 0; i < axes; i++)
        {
            if (!iControlMode->setVelocityMode(i))
            {
                CD_ERROR("setVelocityMode failed\n");
                issueStop();
                return;
            }
        }

        currentJointVels[q] = op(currentJointVels[q], JOINT_VELOCITY_STEP);

        if (std::abs(currentJointVels[q]) > maxVelocityLimits[q])
        {
            CD_WARNING("Absolute joint velocity limit exceeded: maxVelocityLimits[%d] = %f\n", q, maxVelocityLimits[q]);
            currentJointVels[q] = op(0, 1) * maxVelocityLimits[q];
        }

        std::cout << "New joint velocity: " << currentJointVels << std::endl;

        if (!iVelocityControl->velocityMove(q, currentJointVels[q]))
        {
            CD_ERROR("velocityMove failed\n");
        }
    }
    else
    {
        CD_WARNING("Unrecognized key, only %d joints available.\n", axes);
        issueStop();
    }
}

template <typename func>
void roboticslab::RateControllerConsole::incrementOrDecrementCartesianVelocity(cart coord, func op)
{
    bool isLinear = coord == X || coord == Y || coord == Z;
    double step = isLinear ? CARTESIAN_LINEAR_VELOCITY_STEP : CARTESIAN_ANGULAR_VELOCITY_STEP;

    currentCartVels[coord] = op(currentCartVels[coord], step);

    std::cout << "New cartesian velocity: " << currentCartVels << std::endl;

    if (!iCartesianControl->movv(currentCartVels))
    {
        CD_ERROR("movv failed\n");
    }
}

void roboticslab::RateControllerConsole::issueStop()
{
    iCartesianControl->stopControl();
    std::fill(currentJointVels.begin(), currentJointVels.end(), 0.0);
    std::fill(currentCartVels.begin(), currentCartVels.end(), 0.0);
    std::cout << "Stopped" << std::endl;
}

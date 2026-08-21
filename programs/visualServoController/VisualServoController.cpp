// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "VisualServoController.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Value.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_CARTESIAN_DEVICE = "CartesianControlClient";
constexpr auto DEFAULT_LOCAL_PORT = "/VisualServoController";
constexpr auto DEFAULT_REMOTE_VISION = "/haarDetection2D";
constexpr auto DEFAULT_REMOTE_CARTESIAN = "/CartesianControl";
#if 0
constexpr auto DEFAULT_PROXIMITY_SENSORS = "/sensor_reader";
#endif
constexpr auto DEFAULT_PERIOD = 0.01; // [s]

constexpr auto INITIAL_ACT_DELAY = 3; // [s]
constexpr auto FINAL_ACT_DELAY = 5; // [s]

bool VisualServoController::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(VSC) << "Config:" << rf.toString();

    auto cartesianDeviceName = rf.check("cartesianDevice", yarp::os::Value(DEFAULT_CARTESIAN_DEVICE), "cartesian device name").asString();
    auto localPort = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PORT), "local cartesian port").asString();
    auto remoteVision = rf.check("remoteVision", yarp::os::Value(DEFAULT_REMOTE_VISION), "remote vision port").asString();
    auto remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_REMOTE_CARTESIAN), "remote cartesian port").asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD), "period [s]").asFloat64();

    yarp::os::Property cartesianControlClientOptions {
        {"device", yarp::os::Value(cartesianDeviceName)},
        {"local", yarp::os::Value(localPort)},
        {"remote", yarp::os::Value(remoteCartesian)}
    };

    if (!cartesianControlDevice.open(cartesianControlClientOptions))
    {
        yCError(VSC) << "Cartesian control client device not valid";
        return false;
    }

    if (!cartesianControlDevice.view(iCartesianControl))
    {
        yCError(VSC) << "Could not view iCartesianControl";
        return false;
    }

#if 0
    if (!rf.check("disableSensors"))
    {
        auto sensorsPort = rf.check("sensorsPort", yarp::os::Value(DEFAULT_PROXIMITY_SENSORS), "remote sensors port").asString();

        yarp::os::Property sensorsClientOptions;
        sensorsClientOptions.fromString(rf.toString());
        sensorsClientOptions.put("device", "ProximitySensorsClient");
        sensorsClientOptions.put("local", localPort);
        sensorsClientOptions.put("remote", sensorsPort);

        sensorsClientDevice.open(sensorsClientOptions);

        if (!sensorsClientDevice.isValid())
        {
            yCError(VSC) << "Proximity sensors device not valid";
            return false;
        }

        if (!sensorsClientDevice.view(iProximitySensors))
        {
            yCError(VSC) << "Could not view iProximitySensors";
            return false;
        }
    }
#endif

    if (!iCartesianControl->actuateTool(ICartesianControl::Actuator::OPEN))
    {
        yCError(VSC) << "Unable to actuate tool";
        return false;
    }

    if (!rf.check("noMove"))
    {
        grabberResponder.setICartesianControlDriver(iCartesianControl);

        if (!iCartesianControl->setParameter(ICartesianControl::Config::FRAME, static_cast<double>(ICartesianSolver::Frame::TCP)))
        {
            yCError(VSC) << "Unable to set TCP reference frame";
            return false;
        }

        grabberResponder.setNoApproachSetting(rf.check("noApproach"));

        grabberPort.useCallback(grabberResponder);
        grabberPort.open(localPort + "/state:i");

        if (!yarp::os::Network::connect(remoteVision + "/state:o", localPort + "/state:i"))
        {
            yCError(VSC) << "Unable to connect to remote vision port with prefix:" << remoteVision;
            return false;
        }
    }

    yCInfo(VSC) << "Delaying" << INITIAL_ACT_DELAY << "seconds...";
    yarp::os::SystemClock::delaySystem(INITIAL_ACT_DELAY);

    return true;
}

bool VisualServoController::updateModule()
{
#if 0
    if (sensorsClientDevice.isValid() && iProximitySensors->hasTarget())
    {
        yCInfo(VSC) << "Target detected";

        // disable servo control, stop motors and close stream of sensor data
        grabberPort.interrupt();
        iCartesianControl->stopControl();
        sensorsClientDevice.close();

        // close gripper, delay needed for AMOR
        const double now = yarp::os::SystemClock::now();

        while (yarp::os::SystemClock::now() - now < FINAL_ACT_DELAY)
        {
            iCartesianControl->actuateTool(ICartesianControl::Actuator::CLOSE);
            yarp::os::SystemClock::delaySystem(0.1);
        }

        return false;
    }
#endif

    return true;
}

bool VisualServoController::interruptModule()
{
    grabberPort.interrupt();

    if (iCartesianControl)
    {
        iCartesianControl->stopControl();
    }

    return true;
}

bool VisualServoController::close()
{
    grabberPort.close();

    bool ret = true;
#if 0
    ret &= sensorsClientDevice.close();
#endif
    ret &= cartesianControlDevice.close();
    return ret;
}

double VisualServoController::getPeriod()
{
    return period;
}

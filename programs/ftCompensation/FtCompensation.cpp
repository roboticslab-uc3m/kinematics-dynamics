// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FtCompensation.hpp"

#include <functional> // std::invoke

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>

#include <kdl/utilities/utility.h> // KDL::deg2rad

#include "KdlVectorConverter.hpp"

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(FTC, "rl.FtCompensation")
}

constexpr auto DEFAULT_LOCAL_PREFIX = "/ftCompensation";
constexpr auto DEFAULT_PERIOD = 0.02;
constexpr auto DEFAULT_LIN_GAIN = 0.01;
constexpr auto DEFAULT_ROT_GAIN = 0.2;
constexpr auto DEFAULT_FORCE_DEADBAND = 1.0;
constexpr auto DEFAULT_TORQUE_DEADBAND = 1.0;
constexpr auto DEFAULT_LIN_STIFFNESS = 0.1;
constexpr auto DEFAULT_ROT_STIFFNESS = 0.1;
constexpr auto DEFAULT_LIN_DAMPING = 0.1;
constexpr auto DEFAULT_ROT_DAMPING = 0.1;

bool FtCompensation::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(FTC) << "Config:" << rf.toString();

    auto modeStr = rf.check("mode", yarp::os::Value(""), "command mode (twist or wrench)").asString();

    int modeVocab;

    if (modeStr == "twist")
    {
        modeVocab = VOCAB_CC_TWIST;
        command = &ICartesianControl::twist;
    }
    else if (modeStr == "wrench")
    {
        modeVocab = VOCAB_CC_WRENCH;
        command = &ICartesianControl::wrench;
    }
    else
    {
        yCError(FTC) << "Invalid mode:" << modeStr << "(must be 'twist' or 'wrench')";
        return false;
    }

    auto period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD), "period [s]").asFloat64();

    dryRun = rf.check("dryRun", "process sensor loops, but don't send motion command");

    linGain = rf.check("linGain", yarp::os::Value(DEFAULT_LIN_GAIN), "linear gain").asFloat64();
    rotGain = rf.check("rotGain", yarp::os::Value(DEFAULT_ROT_GAIN), "rotational gain").asFloat64();

    linStiffness = rf.check("linStiffness", yarp::os::Value(DEFAULT_LIN_STIFFNESS), "linear stiffness [N/m]").asFloat64();
    rotStiffness = rf.check("rotStiffness", yarp::os::Value(DEFAULT_ROT_STIFFNESS), "rotational stiffness [Nm/rad]").asFloat64();

    linDamping = rf.check("linDamping", yarp::os::Value(DEFAULT_LIN_DAMPING), "linear damping [Ns/m]").asFloat64();
    rotDamping = rf.check("rotDamping", yarp::os::Value(DEFAULT_ROT_DAMPING), "rotational damping [Nms/rad]").asFloat64();

    forceDeadband = rf.check("forceDeadband", yarp::os::Value(DEFAULT_FORCE_DEADBAND), "force deadband [N]").asFloat64();
    torqueDeadband = rf.check("torqueDeadband", yarp::os::Value(DEFAULT_TORQUE_DEADBAND), "torque deadband [Nm]").asFloat64();

    if (rf.check("linStiffness") || rf.check("rotStiffness") || rf.check("linDamping") || rf.check("rotDamping"))
    {
        if (modeVocab != VOCAB_CC_WRENCH)
        {
            yCError(FTC) << "Impedance control can only be enabled in wrench mode";
            return false;
        }

        enableImpedance = true;
    }
    else
    {
        enableImpedance = false;
    }

    yCInfo(FTC) << "Using linear gain:" << linGain;
    yCInfo(FTC) << "Using rotational gain:" << rotGain;

    if (enableImpedance)
    {
        yCInfo(FTC) << "Using linear stiffness [N/m]:" << linStiffness;
        yCInfo(FTC) << "Using rotational stiffness [Nm/rad]:" << rotStiffness;
        yCInfo(FTC) << "Using linear damping [Ns/m]:" << linDamping;
        yCInfo(FTC) << "Using rotational damping [Nms/rad]:" << rotDamping;
    }

    yCInfo(FTC) << "Using force deadband [N]:" << forceDeadband;
    yCInfo(FTC) << "Using torque deadband [Nm]:" << torqueDeadband;

    auto sensorFrameRPY = rf.check("sensorFrameRPY", yarp::os::Value::getNullValue(), "sensor frame RPY rotation regarding TCP frame [deg]");

    if (!sensorFrameRPY.isNull())
    {
        if (!sensorFrameRPY.isList() || sensorFrameRPY.asList()->size() != 3)
        {
            yCError(FTC) << "sensorFrameRPY must be a list of 3 doubles";
            return false;
        }

        yCInfo(FTC) << "Sensor frame RPY [deg]:" << sensorFrameRPY.toString();

        auto roll = sensorFrameRPY.asList()->get(0).asFloat64() * KDL::deg2rad;
        auto pitch = sensorFrameRPY.asList()->get(1).asFloat64() * KDL::deg2rad;
        auto yaw = sensorFrameRPY.asList()->get(2).asFloat64() * KDL::deg2rad;

        // sequence (old axes): 1. R_x(roll), 2. R_y(pitch), 3. R_z(yaw)
        R_N_sensor = KDL::Rotation::RPY(roll, pitch, yaw);
    }
    else
    {
        yCInfo(FTC) << "Using no sensor frame";
        R_N_sensor = KDL::Rotation::Identity();
    }

    auto localPrefix = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PREFIX), "local port prefix").asString();

    // ----- sensor device -----

    if (!rf.check("sensorName", "remote FT sensor name to connect to via MAS client"))
    {
        yCError(FTC) << "Missing parameter: sensorName";
        return false;
    }

    auto sensorName = rf.find("sensorName").asString();

    if (!rf.check("sensorRemote", "remote FT sensor port to connect to via MAS client"))
    {
        yCError(FTC) << "Missing parameter: sensorRemote";
        return false;
    }

    auto sensorRemote = rf.find("sensorRemote").asString();

    yarp::os::Property sensorOptions {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value(sensorRemote)},
        {"local", yarp::os::Value(localPrefix + sensorRemote)},
        {"timeout", yarp::os::Value(period)}
    };

    if (!sensorDevice.open(sensorOptions))
    {
        yCError(FTC) << "Failed to open sensor device";
        return false;
    }

    if (!sensorDevice.view(sensor))
    {
        yCError(FTC) << "Failed to view sensor interface";
        return false;
    }

    sensorIndex = -1;

    for (auto i = 0; i < sensor->getNrOfSixAxisForceTorqueSensors(); i++)
    {
        std::string temp;

        if (sensor->getSixAxisForceTorqueSensorName(i, temp) && temp == sensorName)
        {
            sensorIndex = i;
            break;
        }
    }

    if (sensorIndex == -1)
    {
        yCError(FTC) << "Failed to find sensor with name" << sensorName;
        return false;
    }

    int retry = 0;

    while (sensor->getSixAxisForceTorqueSensorStatus(sensorIndex) != yarp::dev::MAS_OK && retry++ < 10)
    {
        yCDebug(FTC) << "Waiting for sensor to be ready... retry" << retry;
        yarp::os::SystemClock::delaySystem(0.1);
    }

    if (retry == 10)
    {
        yCError(FTC) << "Failed to get first read, max number of retries exceeded";
        return false;
    }

    if (!readSensor(initialOffset))
    {
        yCError(FTC) << "Failed to read sensor";
        return false;
    }

    // ----- tool compensation -----

    auto vToolCoM = rf.check("toolCoM", yarp::os::Value::getNullValue(), "tool CoM regarding TCP frame");
    auto vToolWeight = rf.check("toolWeight", yarp::os::Value::getNullValue(), "tool weight vector regarding inertial frame");

    if (!vToolCoM.isNull() && !vToolWeight.isNull())
    {
        if (!vToolCoM.isList() || vToolCoM.asList()->size() != 3)
        {
            yCError(FTC) << "toolCoM must be a list of 3 doubles";
            return false;
        }

        if (!vToolWeight.isList() || vToolWeight.asList()->size() != 3)
        {
            yCError(FTC) << "toolWeight must be a list of 3 doubles";
            return false;
        }

        yCInfo(FTC) << "Tool CoM regarding TCP frame:" << vToolCoM.toString();

        toolCoM_N.x(vToolCoM.asList()->get(0).asFloat64());
        toolCoM_N.y(vToolCoM.asList()->get(1).asFloat64());
        toolCoM_N.z(vToolCoM.asList()->get(2).asFloat64());

        yCInfo(FTC) << "Tool weight regarding inertial frame:" << vToolWeight.toString();

        toolWeight_0.force.x(vToolWeight.asList()->get(0).asFloat64());
        toolWeight_0.force.y(vToolWeight.asList()->get(1).asFloat64());
        toolWeight_0.force.z(vToolWeight.asList()->get(2).asFloat64());
        toolWeight_0.torque = KDL::Vector::Zero();

        usingTool = true;
    }
    else
    {
        yCInfo(FTC) << "Using no tool";
        usingTool = false;
    }

    // ----- cartesian device -----

    if (!dryRun || usingTool || enableImpedance)
    {
        if (!rf.check("cartesianRemote", "remote cartesian port to connect to"))
        {
            yCError(FTC) << "Missing parameter: cartesianRemote";
            return false;
        }

        auto cartesianRemote = rf.find("cartesianRemote").asString();

        yarp::os::Property cartesianOptions {
            {"device", yarp::os::Value("CartesianControlClient")},
            {"cartesianRemote", yarp::os::Value(cartesianRemote)},
            {"cartesianLocal", yarp::os::Value(localPrefix + cartesianRemote)}
        };

        if (!cartesianDevice.open(cartesianOptions))
        {
            yCError(FTC) << "Failed to open cartesian device";
            return false;
        }

        if (!cartesianDevice.view(iCartesianControl))
        {
            yCError(FTC) << "Failed to view cartesian control interface";
            return false;
        }

        if (!dryRun)
        {
            std::map<int, double> params;

            if (!iCartesianControl->getParameters(params))
            {
                yCError(FTC) << "Unable to retrieve cartesian configuration parameters";
                return false;
            }

            bool usingStreamingPreset = params.find(VOCAB_CC_CONFIG_STREAMING_CMD) != params.end();

            if (usingStreamingPreset && !iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, modeVocab))
            {
                yCWarning(FTC) << "Unable to preset streaming command";
                return false;
            }

            if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::TCP_FRAME))
            {
                yCWarning(FTC) << "Unable to set TCP frame";
                return false;
            }
        }

        if (usingTool && !compensateTool(initialOffset))
        {
            yCError(FTC) << "Failed to compensate tool";
            return false;
        }

        std::vector<double> x;

        if (enableImpedance && !iCartesianControl->stat(x))
        {
            yCError(FTC) << "Failed to retrieve initial pose";
            return false;
        }

        initialPose = KdlVectorConverter::vectorToFrame(x);
        previousPose = initialPose;
    }
    else if (dryRun)
    {
        yCInfo(FTC) << "Dry run mode enabled, robot will perform no motion";
    }

    yarp::os::PeriodicThread::setPeriod(period);

    return yarp::os::PeriodicThread::start();
}

bool FtCompensation::readSensor(KDL::Wrench & wrench) const
{
    yarp::sig::Vector outSensor;

    if (double timestamp; !sensor->getSixAxisForceTorqueSensorMeasure(sensorIndex, outSensor, timestamp))
    {
        yCWarning(FTC) << "Failed to retrieve current sensor measurements";
        return false;
    }

    KDL::Wrench currentWrench_sensor;
    currentWrench_sensor.force = {outSensor[0], outSensor[1], outSensor[2]};
    currentWrench_sensor.torque = {outSensor[3], outSensor[4], outSensor[5]};

    wrench = R_N_sensor * currentWrench_sensor;
    return true;
}

bool FtCompensation::compensateTool(KDL::Wrench & wrench) const
{
    std::vector<double> currentX;

    if (!iCartesianControl->stat(currentX))
    {
        yCWarning(FTC) << "Failed to retrieve current position";
        return false;
    }

    auto H_0_N = KdlVectorConverter::vectorToFrame(currentX);
    auto toolWrench = H_0_N.M.Inverse() * toolWeight_0;
    auto toolWrench_N = toolWrench.RefPoint(-toolCoM_N);

    wrench -= toolWrench_N;
    return true;
}

bool FtCompensation::applyImpedance(KDL::Wrench & wrench)
{
    std::vector<double> x;

    if (!iCartesianControl->stat(x))
    {
        yCWarning(FTC) << "Failed to retrieve current position";
        return false;
    }

    KDL::Frame currentPose = KdlVectorConverter::vectorToFrame(x);
    KDL::Twist displacement = KDL::diff(initialPose, currentPose);
    KDL::Twist twist = KDL::diff(previousPose, currentPose, yarp::os::PeriodicThread::getPeriod());

    wrench.force -= displacement.vel * linStiffness + twist.vel * linDamping;
    wrench.torque -= displacement.rot * rotStiffness + twist.rot * rotDamping;

    previousPose = currentPose;
    return true;
}

void FtCompensation::run()
{
    KDL::Wrench wrench;

    if (!readSensor(wrench) || (usingTool && !compensateTool(wrench)))
    {
        yarp::os::PeriodicThread::askToStop();
        return;
    }

    wrench -= initialOffset;

    auto forceNorm = wrench.force.Norm();
    auto torqueNorm = wrench.torque.Norm();

    if (forceNorm > forceDeadband)
    {
        auto forceThreshold = (wrench.force / forceNorm) * forceDeadband;
        wrench.force = (wrench.force - forceThreshold) * linGain;
    }
    else
    {
        wrench.force = KDL::Vector::Zero();
    }

    if (torqueNorm > torqueDeadband)
    {
        auto torqueThreshold = (wrench.torque / torqueNorm) * torqueDeadband;
        wrench.torque = (wrench.torque - torqueThreshold) * rotGain;
    }
    else
    {
        wrench.torque = KDL::Vector::Zero();
    }

    if (enableImpedance && !applyImpedance(wrench))
    {
        yarp::os::PeriodicThread::askToStop();
        return;
    }

    auto v = KdlVectorConverter::wrenchToVector(wrench);
    yCDebug(FTC) << v;

    if (!dryRun)
    {
        std::invoke(command, iCartesianControl, v);
    }
}

bool FtCompensation::updateModule()
{
    return yarp::os::PeriodicThread::isRunning();
}

bool FtCompensation::interruptModule()
{
    yarp::os::PeriodicThread::stop();
    return dryRun || iCartesianControl->stopControl();
}

double FtCompensation::getPeriod()
{
    return 1.0; // [s]
}

bool FtCompensation::close()
{
    cartesianDevice.close();
    sensorDevice.close();
    return true;
}

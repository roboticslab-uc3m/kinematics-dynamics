// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <string>

#include <yarp/conf/version.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include "KinematicRepresentation.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_CAN_LIBRARY = "libeddriver.so";
constexpr auto DEFAULT_CAN_PORT = 0;
constexpr auto DEFAULT_GAIN = 0.05;
constexpr auto DEFAULT_WAIT_PERIOD_MS = 30;
constexpr auto DEFAULT_REFERENCE_FRAME = "base";

// ------------------- DeviceDriver Related ------------------------------------

bool AmorCartesianControl::open(yarp::os::Searchable& config)
{
    gain = config.check("controllerGain", yarp::os::Value(DEFAULT_GAIN),
            "controller gain").asFloat64();

    waitPeriodMs = config.check("waitPeriodMs", yarp::os::Value(DEFAULT_WAIT_PERIOD_MS),
            "wait command period (milliseconds)").asInt32();

    auto referenceFrameStr = config.check("referenceFrame", yarp::os::Value(DEFAULT_REFERENCE_FRAME),
            "reference frame (base|tcp)").asString();

    if (referenceFrameStr == "base")
    {
        referenceFrame = ICartesianSolver::BASE_FRAME;
    }
    else if (referenceFrameStr == "tcp")
    {
        referenceFrame = ICartesianSolver::TCP_FRAME;
    }
    else
    {
        yCError(AMOR) << "Unsupported reference frame:" << referenceFrameStr;
        return false;
    }

    yarp::os::Value vHandle = config.find("handle");
    yarp::os::Value vHandleMutex = config.find("handleMutex");

    if (vHandle.isNull() || vHandleMutex.isNull())
    {
        yCInfo(AMOR) << "Creating own AMOR handle";

        auto canLibrary = config.check("canLibrary", yarp::os::Value(DEFAULT_CAN_LIBRARY),
                "CAN plugin library").asString();
        int canPort = config.check("canPort", yarp::os::Value(DEFAULT_CAN_PORT),
                "CAN port number").asInt32();

        ownsHandle = true;
        handle = amor_connect(const_cast<char *>(canLibrary.c_str()), canPort);
        handleMutex = new std::mutex;
    }
    else
    {
        yCInfo(AMOR) << "Using external AMOR handle";
        ownsHandle = false;
        handle = *reinterpret_cast<AMOR_HANDLE *>(const_cast<char *>(vHandle.asBlob()));
        handleMutex = reinterpret_cast<std::mutex *>(const_cast<char *>(vHandleMutex.asBlob()));
    }

    if (std::lock_guard<std::mutex> lock(*handleMutex); handle == AMOR_INVALID_HANDLE)
    {
        yCError(AMOR) << "Could not get AMOR handle:" << amor_error();
        return false;
    }

    qdotMax.resize(AMOR_NUM_JOINTS);

    yarp::os::Bottle qMin, qMax;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        AMOR_JOINT_INFO jointInfo;

        if (std::lock_guard<std::mutex> lock(*handleMutex); amor_get_joint_info(handle, i, &jointInfo) != AMOR_SUCCESS)
        {
            yCError(AMOR) << "amor_get_joint_info() failed:" << amor_error();
            return false;
        }

        qdotMax[i] = KinRepresentation::radToDeg(jointInfo.maxVelocity);

        qMin.addFloat64(KinRepresentation::radToDeg(jointInfo.lowerJointLimit));
        qMax.addFloat64(KinRepresentation::radToDeg(jointInfo.upperJointLimit));
    }

    auto kinematicsFile = config.check("kinematics", yarp::os::Value(""),
            "path to file with description of AMOR kinematics").asString();

    yarp::os::Property cartesianDeviceOptions;

    if (!cartesianDeviceOptions.fromConfigFile(kinematicsFile))
    {
        yCError(AMOR) << "Cannot read from --kinematics" << kinematicsFile;
        return false;
    }

    std::string solverStr = "KdlSolver";
    cartesianDeviceOptions.put("device", solverStr);
    cartesianDeviceOptions.put("mins", yarp::os::Value::makeList(qMin.toString().c_str()));
    cartesianDeviceOptions.put("maxs", yarp::os::Value::makeList(qMax.toString().c_str()));
    cartesianDeviceOptions.setMonitor(config.getMonitor(), solverStr.c_str());

    if (!cartesianDevice.open(cartesianDeviceOptions))
    {
        yCError(AMOR) << "Solver device not valid";
        return false;
    }

    if (!cartesianDevice.view(iCartesianSolver))
    {
        yCError(AMOR) << "Could not view iCartesianSolver";
        return false;
    }

    currentState = VOCAB_CC_NOT_CONTROLLING;
    return true;
}

// -----------------------------------------------------------------------------

bool AmorCartesianControl::close()
{
    if (handle != AMOR_INVALID_HANDLE)
    {
        std::unique_lock<std::mutex> lock(*handleMutex);
        amor_emergency_stop(handle);

        if (ownsHandle)
        {
            amor_release(handle);
            lock.unlock();
            delete handleMutex;
        }
    }

    handle = AMOR_INVALID_HANDLE;
    handleMutex = nullptr;

    return cartesianDevice.close();
}

// -----------------------------------------------------------------------------

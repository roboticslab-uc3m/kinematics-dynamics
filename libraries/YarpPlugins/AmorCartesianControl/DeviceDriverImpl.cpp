// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include "KinematicRepresentation.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AmorCartesianControl::open(yarp::os::Searchable& config)
{
    yDebug() << "AmorCartesianControl config:" << config.toString();

    gain = config.check("controllerGain", yarp::os::Value(DEFAULT_GAIN),
            "controller gain").asFloat64();

    waitPeriodMs = config.check("waitPeriodMs", yarp::os::Value(DEFAULT_WAIT_PERIOD_MS),
            "wait command period (milliseconds)").asInt32();

    std::string referenceFrameStr = config.check("referenceFrame", yarp::os::Value(DEFAULT_REFERENCE_FRAME),
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
        yError() << "Unsupported reference frame:" << referenceFrameStr;
        return false;
    }

    yarp::os::Value vHandle = config.find("handle");

    if (vHandle.isNull())
    {
        yInfo() << "Creating own AMOR handle";

        std::string canLibrary = config.check("canLibrary", yarp::os::Value(DEFAULT_CAN_LIBRARY),
                "CAN plugin library").asString();
        int canPort = config.check("canPort", yarp::os::Value(DEFAULT_CAN_PORT),
                "CAN port number").asInt32();

        ownsHandle = true;
        handle = amor_connect(const_cast<char *>(canLibrary.c_str()), canPort);
    }
    else
    {
        yInfo() << "Using external AMOR handle";
        ownsHandle = false;
        handle = *(reinterpret_cast<AMOR_HANDLE *>(const_cast<char *>(vHandle.asBlob())));
    }

    if (handle == AMOR_INVALID_HANDLE)
    {
        yError() << "Could not get AMOR handle:" << amor_error();
        close();
        return false;
    }

    qdotMax.resize(AMOR_NUM_JOINTS);

    yarp::os::Bottle qMin, qMax;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        AMOR_JOINT_INFO jointInfo;

        if (amor_get_joint_info(handle, i, &jointInfo) != AMOR_SUCCESS)
        {
            yError() << "amor_get_joint_info() failed:" << amor_error();
            close();
            return false;
        }

        qdotMax[i] = KinRepresentation::radToDeg(jointInfo.maxVelocity);

        qMin.addFloat64(KinRepresentation::radToDeg(jointInfo.lowerJointLimit));
        qMax.addFloat64(KinRepresentation::radToDeg(jointInfo.upperJointLimit));
    }

    std::string kinematicsFile = config.check("kinematics", yarp::os::Value(""),
            "path to file with description of AMOR kinematics").asString();

    yarp::os::Property cartesianDeviceOptions;

    if (!cartesianDeviceOptions.fromConfigFile(kinematicsFile))
    {
        yError() << "Cannot read from --kinematics" << kinematicsFile;
        return false;
    }

    std::string solverStr = "KdlSolver";
    cartesianDeviceOptions.put("device", solverStr);
    cartesianDeviceOptions.put("mins", yarp::os::Value::makeList(qMin.toString().c_str()));
    cartesianDeviceOptions.put("maxs", yarp::os::Value::makeList(qMax.toString().c_str()));
    cartesianDeviceOptions.setMonitor(config.getMonitor(), solverStr.c_str());

    if (!cartesianDevice.open(cartesianDeviceOptions))
    {
        yError() << "Solver device not valid";
        return false;
    }

    if (!cartesianDevice.view(iCartesianSolver))
    {
        yError() << "Could not view iCartesianSolver";
        close();
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::close()
{
    if (handle != AMOR_INVALID_HANDLE)
    {
        amor_emergency_stop(handle);
    }

    if (ownsHandle)
    {
        amor_release(handle);
    }

    handle = AMOR_INVALID_HANDLE;

    return cartesianDevice.close();
}

// -----------------------------------------------------------------------------

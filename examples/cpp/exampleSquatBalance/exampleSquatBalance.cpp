// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup kinematics-dynamics-examples
 * \defgroup exampleSquatBalance exampleSquatBalance
 */

#include <string>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>

#include <ICartesianControl.h>
#include <KdlTrajectory.hpp>
#include <ColorDebug.h>

#include "TrajectoryThread.hpp"

#define TRAJ_DURATION 10.0
#define TRAJ_MAX_VEL 0.05
#define PT_MODE_MS 50.0

namespace rl = roboticslab;

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR("Please start a yarp name server first.\n");
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    std::string robotPrefix = rf.check("prefix", yarp::os::Value("/teoSim")).asString();

    double y = rf.check("y", yarp::os::Value(0.0)).asFloat64();
    double z = rf.check("z", yarp::os::Value(0.0)).asFloat64();

    // Create devices.

    yarp::os::Property leftLegDeviceOptions;
    leftLegDeviceOptions.put("device", "CartesianControlClient");
    leftLegDeviceOptions.put("cartesianRemote", robotPrefix + "/CartesianControl/leftLeg");
    leftLegDeviceOptions.put("cartesianLocal", "/screwTheoryTrajectoryExample/leftLeg");

    yarp::dev::PolyDriver leftLegDevice(leftLegDeviceOptions);

    if (!leftLegDevice.isValid())
    {
        CD_ERROR("Cartesian device (left leg) not available.\n");
        return 1;
    }

    rl::ICartesianControl * iCartesianControlLeftLeg;

    if (!leftLegDevice.view(iCartesianControlLeftLeg))
    {
        CD_ERROR("Cannot view iCartesianControlLeftLeg.\n");
        return 1;
    }

    yarp::os::Property rightLegDeviceOptions;
    rightLegDeviceOptions.put("device", "CartesianControlClient");
    rightLegDeviceOptions.put("cartesianRemote", robotPrefix + "/CartesianControl/rightLeg");
    rightLegDeviceOptions.put("cartesianLocal", "/screwTheoryTrajectoryExample/rightLeg");

    yarp::dev::PolyDriver rightLegDevice(rightLegDeviceOptions);

    if (!rightLegDevice.isValid())
    {
        CD_ERROR("Cartesian device (right leg) not available.\n");
        return 1;
    }

    rl::ICartesianControl * iCartesianControlRightLeg;

    if (!rightLegDevice.view(iCartesianControlRightLeg))
    {
        CD_ERROR("Cannot view iCartesianControlRightLeg.\n");
        return 1;
    }

    // Configure trajectories.

    std::vector<double> x_leftLeg;

    if (!iCartesianControlLeftLeg->stat(x_leftLeg))
    {
        CD_ERROR("stat() failed (left leg).\n");
        return 1;
    }

    std::vector<double> xd_leftLeg(x_leftLeg);
    xd_leftLeg[1] -= y;
    xd_leftLeg[2] += z;

    CD_INFO("Current (left): %f %f %f\n", x_leftLeg[0], x_leftLeg[1], x_leftLeg[2]);
    CD_INFO("Desired (left): %f %f %f\n", xd_leftLeg[0], xd_leftLeg[1], xd_leftLeg[2]);

    rl::KdlTrajectory trajectoryLeftLeg;

    trajectoryLeftLeg.setDuration(TRAJ_DURATION);
    trajectoryLeftLeg.setMaxVelocity(TRAJ_MAX_VEL);
    trajectoryLeftLeg.addWaypoint(x_leftLeg);
    trajectoryLeftLeg.addWaypoint(xd_leftLeg);
    trajectoryLeftLeg.configurePath(rl::ICartesianTrajectory::LINE);
    trajectoryLeftLeg.configureVelocityProfile(rl::ICartesianTrajectory::TRAPEZOIDAL);

    if (!trajectoryLeftLeg.create())
    {
        CD_ERROR("Problem creating cartesian trajectory (left leg).\n");
        return 1;
    }

    std::vector<double> x_rightLeg;

    if (!iCartesianControlRightLeg->stat(x_rightLeg))
    {
        CD_ERROR("stat() failed (right leg).\n");
        return 1;
    }

    std::vector<double> xd_rightLeg(x_rightLeg);
    xd_rightLeg[1] -= y;
    xd_rightLeg[2] += z;

    CD_INFO("Current (right): %f %f %f\n", x_rightLeg[0], x_rightLeg[1], x_rightLeg[2]);
    CD_INFO("Desired (right): %f %f %f\n", xd_rightLeg[0], xd_rightLeg[1], xd_rightLeg[2]);

    rl::KdlTrajectory trajectoryRightLeg;

    trajectoryRightLeg.setDuration(TRAJ_DURATION);
    trajectoryRightLeg.setMaxVelocity(TRAJ_MAX_VEL);
    trajectoryRightLeg.addWaypoint(x_rightLeg);
    trajectoryRightLeg.addWaypoint(xd_rightLeg);
    trajectoryRightLeg.configurePath(rl::ICartesianTrajectory::LINE);
    trajectoryRightLeg.configureVelocityProfile(rl::ICartesianTrajectory::TRAPEZOIDAL);

    if (!trajectoryRightLeg.create())
    {
        CD_ERROR("Problem creating cartesian trajectory (right leg).\n");
        return 1;
    }

    // Configure workers.

    if (!iCartesianControlLeftLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        CD_ERROR("Cannot preset streaming command (left leg).\n");
        return 1;
    }

    if (!iCartesianControlRightLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        CD_ERROR("Cannot preset streaming command (right leg).\n");
        return 1;
    }

    TrajectoryThread trajThreadLeftLeg(iCartesianControlLeftLeg, &trajectoryLeftLeg, PT_MODE_MS);

    TrajectoryThread trajThreadRightLeg(iCartesianControlRightLeg, &trajectoryRightLeg, PT_MODE_MS);

    // Perform actions.

    if (trajThreadLeftLeg.start() && trajThreadRightLeg.start())
    {
        yarp::os::Time::delay(TRAJ_DURATION);
        trajThreadLeftLeg.stop();
        trajThreadRightLeg.stop();
    }

    leftLegDevice.close();
    rightLegDevice.close();

    return 0;
}

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup kinematics-dynamics-examples
 * \defgroup screwTheoryTrajectoryExample screwTheoryTrajectoryExample
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2018 Universidad Carlos III de Madrid;
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd examples/cpp/exampleScrewTheoryTrajectory/
mkdir build; cd build; cmake ..
make -j$(nproc)
\endverbatim
 * <b>Running example with teoSim</b>
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * What mostly changes is the library command line invocation. We also change the server port name. The following is an example for the simulated robot's right arm.
\verbatim
[on terminal 2] teoSim
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/leftArm/CartesianControl --kinematics teo-fixedTrunk-leftArm-fetch.ini --robot remote_controlboard --local /BasicCartesianControl/teoSim/leftArm --remote /teoSim/leftArm --angleRepr axisAngle
[on terminal 4] ./exampleScrewTheoryTrajectory
\endverbatim
 */

#include <memory>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>
#include <kdl/utilities/utility.h>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <ConfigurationSelector.hpp>
#include <MatrixExponential.hpp>
#include <ProductOfExponentials.hpp>
#include <ScrewTheoryIkProblem.hpp>

#include "TrajectoryThread.hpp"

constexpr auto DEFAULT_REMOTE_PORT = "/teoSim/leftArm";
constexpr auto DEFAULT_TRAJ_DURATION = 10.0;
constexpr auto DEFAULT_TRAJ_MAX_VEL = 0.05;
constexpr auto DEFAULT_PERIOD_MS = 50.0;

namespace rl = roboticslab;

namespace
{
    rl::PoeExpression makeTeoLeftArmKinematics()
    {
        KDL::Frame H_S_0(KDL::Rotation::RotY(-KDL::PI_2) * KDL::Rotation::RotX(-KDL::PI_2), {0, 0.34692, 0.1932 + 0.305});
        KDL::Frame H_0_T({-0.63401, 0, 0});

        rl::PoeExpression poe(H_0_T);

        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, {0, 0, 1}, KDL::Vector::Zero()));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, {0, 1, 0}, KDL::Vector::Zero()));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, {1, 0, 0}, KDL::Vector::Zero()));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, {0, 0, 1}, {-0.32901, 0, 0}));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, {1, 0, 0}, {-0.32901, 0, 0}));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, {0, 0, 1}, {-0.54401, 0, 0}));

        poe.changeBaseFrame(H_S_0);

        return poe;
    }
}

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    std::string remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE_PORT), "remote port").asString();
    double trajDuration = rf.check("trajDuration", yarp::os::Value(DEFAULT_TRAJ_DURATION), "trajectory duration (s)").asFloat64();
    double trajMaxVel = rf.check("trajMaxVel", yarp::os::Value(DEFAULT_TRAJ_MAX_VEL), "trajectory max velocity (m/s)").asFloat64();
    int periodMs = rf.check("periodMs", yarp::os::Value(DEFAULT_PERIOD_MS), "command send period (ms)").asInt32();

    yarp::os::Property jointDeviceOptions {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(remote)},
        {"local", yarp::os::Value("/screwTheoryTrajectoryExample" + remote)}
    };

    yarp::dev::PolyDriver jointDevice(jointDeviceOptions);

    if (!jointDevice.isValid())
    {
        yError() << "Joint device not available";
        return 1;
    }

    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IControlLimits * iControlLimits;
    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IPositionDirect * iPositionDirect;

    if (!jointDevice.view(iEncoders) || !jointDevice.view(iControlLimits)
            || !jointDevice.view(iControlMode) || !jointDevice.view(iPositionDirect))
    {
        yError() << "Problems acquiring joint interfaces";
        return 1;
    }

    int axes;

    if (!iEncoders->getAxes(&axes))
    {
        yError() << "getAxes() failed";
        return 1;
    }

    std::vector<double> q(axes); // tested at: set poss (0 0 0 -90 0 0)

    while (!iEncoders->getEncoders(q.data()))
    {
        yarp::os::Time::delay(0.1);
    }

    rl::PoeExpression poe = makeTeoLeftArmKinematics();

    axes = poe.size(); // just for real TEO (7 joints, 6 motor axes)

    KDL::JntArray jntArray(axes);

    for (int i = 0; i < axes; i++)
    {
        jntArray(i) = KDL::deg2rad * q[i];
    }

    KDL::Frame H_base_start;

    if (!poe.evaluate(jntArray, H_base_start))
    {
        yError() << "FK error";
        return 1;
    }

    rl::ScrewTheoryIkProblemBuilder builder(poe);
    std::unique_ptr<rl::ScrewTheoryIkProblem> ikProblem(builder.build());

    if (!ikProblem.get())
    {
        yError() << "Unable to solve IK";
        return 1;
    }

    KDL::JntArray qMin(axes);
    KDL::JntArray qMax(axes);

    for (int i = 0; i < axes; i++)
    {
        if (!iControlLimits->getLimits(i, &qMin(i), &qMax(i)))
        {
            yError() << "Unable to retrieve limits for joint" << i;
            return 1;
        }
    }

    rl::ConfigurationSelectorLeastOverallAngularDisplacementFactory confFactory(qMin, qMax);
    std::unique_ptr<rl::ConfigurationSelector> ikConfig(confFactory.create());

    KDL::Frame H_base_end = H_base_start;
    H_base_end.p += {0.15, 0.1, 0.1};

    KDL::RotationalInterpolation_SingleAxis interp;
    KDL::Path_Line path(H_base_start, H_base_end, &interp, 0.1, false);
    KDL::VelocityProfile_Trap profile(trajMaxVel, 0.2);
    KDL::Trajectory_Segment trajectory(&path, &profile, trajDuration, false);

    std::vector<int> modes(axes, VOCAB_CM_POSITION_DIRECT);

    if (!iControlMode->setControlModes(modes.data()))
    {
        yError() << "Unable to change mode";
        return 1;
    }

    TrajectoryThread trajThread(iEncoders, iPositionDirect, ikProblem.get(), ikConfig.get(), &trajectory, periodMs);

    if (trajThread.start())
    {
        yarp::os::Time::delay(trajDuration);
        trajThread.stop();
    }

    jointDevice.close();

    return 0;
}

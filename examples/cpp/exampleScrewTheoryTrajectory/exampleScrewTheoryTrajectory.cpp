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
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/lefttArm/CartesianControl --from /usr/local/share/teo-configuration-files/contexts/kinematics/lefttArmKinematics.ini --robot remote_controlboard --local /BasicCartesianControl/teoSim/rightArm --remote /teoSim/rightArm --angleRepr axisAngle
[on terminal 4] ./exampleScrewTheoryTrajectory
\endverbatim
 */

#include <memory>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include <ColorDebug.h>

#include <ConfigurationSelector.hpp>
#include <KdlTrajectory.hpp>
#include <KdlVectorConverter.hpp>
#include <KinematicRepresentation.hpp>
#include <MatrixExponential.hpp>
#include <ProductOfExponentials.hpp>
#include <ScrewTheoryIkProblem.hpp>

#include "TrajectoryThread.hpp"

#define TRAJ_DURATION 10.0
#define TRAJ_MAX_VEL 0.05
#define PT_MODE_MS 50.0

namespace rl = roboticslab;

namespace
{
    rl::PoeExpression makeTeoLeftArmKinematics()
    {
        KDL::Frame H_S_0(KDL::Rotation::RotY(KDL::PI / 2) * KDL::Rotation::RotX(-KDL::PI / 2), KDL::Vector(0, 0.34692, 0.1932 + 0.305));
        KDL::Frame H_0_T(KDL::Rotation::RotX(KDL::PI / 2), KDL::Vector(0.718506, 0, 0));

        rl::PoeExpression poe(H_0_T);

        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector::Zero()));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, KDL::Vector(0, -1, 0), KDL::Vector::Zero()));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, KDL::Vector(1,  0, 0), KDL::Vector::Zero()));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(0.32901, 0, 0)));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, KDL::Vector(1,  0, 0), KDL::Vector(0.32901, 0, 0)));
        poe.append(rl::MatrixExponential(rl::MatrixExponential::ROTATION, KDL::Vector(0,  0, 1), KDL::Vector(0.53101, 0, 0)));

        poe.changeBaseFrame(H_S_0);

        return poe;
    }
}

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR("Please start a yarp name server first.\n");
        return 1;
    }

    yarp::os::Property jointDeviceOptions;
    jointDeviceOptions.put("device", "remote_controlboard");
    jointDeviceOptions.put("remote", "/teoSim/leftArm"); // use /teo for real robot
    jointDeviceOptions.put("local", "/screwTheoryTrajectoryExample");

    yarp::dev::PolyDriver jointDevice(jointDeviceOptions);

    if (!jointDevice.isValid())
    {
        CD_ERROR("Joint device not available.\n");
        return 1;
    }

    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IControlLimits2 * iControlLimits;
    yarp::dev::IControlMode2 * iControlMode;
    yarp::dev::IPositionDirect * iPositionDirect;

    if (!jointDevice.view(iEncoders) || !jointDevice.view(iControlLimits)
            || !jointDevice.view(iControlMode) || !jointDevice.view(iPositionDirect))
    {
        CD_ERROR("Problems acquiring joint interfaces.\n");
        return 1;
    }

    int axes;

    if (!iEncoders->getAxes(&axes))
    {
        CD_ERROR("getAxes() failed.\n");
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
        jntArray(i) = rl::KinRepresentation::degToRad(q[i]);
    }

    KDL::Frame H;

    if (!poe.evaluate(jntArray, H))
    {
        CD_ERROR("FK error.\n");
        return 1;
    }

    rl::ScrewTheoryIkProblemBuilder builder(poe);
    std::auto_ptr<rl::ScrewTheoryIkProblem> ikProblem(builder.build());

    if (!ikProblem.get())
    {
        CD_ERROR("Unable to solve IK.\n");
        return 1;
    }

    KDL::JntArray qMin(axes);
    KDL::JntArray qMax(axes);

    for (int i = 0; i < axes; i++)
    {
        if (!iControlLimits->getLimits(i, &qMin(i), &qMax(i)))
        {
            CD_ERROR("Unable to retrieve limits for joint %d.\n", i);
            return 1;
        }
    }

    rl::ConfigurationSelectorLeastOverallAngularDisplacementFactory confFactory(qMin, qMax);
    std::auto_ptr<rl::ConfigurationSelector> ikConfig(confFactory.create());

    std::vector<double> x = rl::KdlVectorConverter::frameToVector(H);

    std::vector<double> xd(x);
    xd[0] += 0.15;
    xd[1] += 0.1;
    xd[2] += 0.1;

    rl::KdlTrajectory trajectory;

    trajectory.setDuration(TRAJ_DURATION);
    trajectory.setMaxVelocity(TRAJ_MAX_VEL);
    trajectory.addWaypoint(x);
    trajectory.addWaypoint(xd);
    trajectory.configurePath(rl::ICartesianTrajectory::LINE);
    trajectory.configureVelocityProfile(rl::ICartesianTrajectory::TRAPEZOIDAL);

    if (!trajectory.create())
    {
        CD_ERROR("Problem creating cartesian trajectory.\n");
        return 1;
    }

    std::vector<int> modes(axes, VOCAB_CM_POSITION_DIRECT);

    if (!iControlMode->setControlModes(modes.data()))
    {
        CD_ERROR("Unable to change mode.\n");
        return 1;
    }

    TrajectoryThread trajThread(iEncoders, iPositionDirect, ikProblem.get(), ikConfig.get(), &trajectory, PT_MODE_MS);

    if (trajThread.start())
    {
        yarp::os::Time::delay(TRAJ_DURATION);
        trajThread.stop();
    }

    jointDevice.close();

    return 0;
}

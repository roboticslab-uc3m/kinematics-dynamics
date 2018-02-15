// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup kinematics-dynamics-examples
 * \defgroup cartesianControlExample cartesianControlExample
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2017 Universidad Carlos III de Madrid;
 *
 * Authors: Raul de Santos Rico
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd examples/cpp/cartesianControlExample/
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
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/rightArm/CartesianControl --from /usr/local/share/teo-configuration-files/contexts/kinematics/rightArmKinematics.ini --robot remote_controlboard --local /BasicCartesianControl/teoSim/rightArm --remote /teoSim/rightArm --angleRepr axisAngle
[on terminal 4] ./cartesianControlExample
[on possible terminal 5] yarp rpc /teoSim/rightArm/CartesianControl/rpc_transform:s (for manual operation)
\endverbatim
 */

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "ICartesianControl.h" // we need this to work with the CartesianControlClient device

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        printf("Please start a yarp name server first\n");
        return 1;
    }

    yarp::os::Property options;
    options.put("device", "CartesianControlClient"); // our device (a dynamically loaded library)
    options.put("cartesianRemote", "/teoSim/rightArm/CartesianControl"); // remote port through which we'll talk to the server
    options.put("cartesianLocal", "/CartesianControlExample");
    options.put("transform", 1);  // Was yarp::os::Value::getNullValue()

    yarp::dev::PolyDriver dd(options);
    if (!dd.isValid())
    {
        printf("Device not available.\n");
        dd.close();
        return 1;
    }

    roboticslab::ICartesianControl *iCartesianControl;

    if (!dd.view(iCartesianControl))
    {
        printf("[error] Problems acquiring interface\n");
        dd.close();
        return 1;
    }
    printf("[success] acquired interface\n");

    std::vector<double> vector;
    int status;

    iCartesianControl->stat(status, vector);
    printf("Controller status (forward kinematics): ");
    for (int i = 0; i < vector.size(); i++)
    {
        printf("%f ", vector[i]);
    }
    printf("\n");

    // -- Move arm in joint space:
    // -- set poss (0 0 0 90 0 0 0)

    // -- Position 1
    std::vector<double> position;
    position.push_back(0.390926); // 0.390926 -0.346663 0.166873 -0.004334 0.70944 0.704752 0.353119
    position.push_back(-0.346663);
    position.push_back(0.166873);
    position.push_back(-0.004334);
    position.push_back(0.70944);
    position.push_back(0.704752);
    position.push_back(0.353119);

    // movj -> go to end position in joint space
    // movl -> go to end position in task space
    printf("Position 1: poss (0 0 0 90 0 0 0)\n");
    iCartesianControl->movj(position);
    iCartesianControl->wait();


    // -- Position 2: move forward along axis X
    printf("Position 2: move forward along axis X\n");
    position[0] = 0.5;
    iCartesianControl->movj(position);
    iCartesianControl->wait();

    // -- Position 3: move right along axis Y
    printf("Position 3: move right along axis Y\n");
    position[1] = -0.4;
    iCartesianControl->movj(position);
    iCartesianControl->wait();

    // -- Position 4: rotate -12 degrees about axis Y
    printf("Position 4: rotate -12 degrees about axis Y\n");
    position[3] = 0.0;
    position[4] = 1.0;
    position[5] = 0.0;
    position[6] = -12.0;
    iCartesianControl->movj(position);
    iCartesianControl->wait();

    // -- Position 5: rotate 50 degrees about axis X
    printf("Position 5: rotate 50 degrees about axis X\n");
    position[3] = 1.0;
    position[4] = 0.0;
    position[5] = 0.0;
    position[6] = -50.0;
    iCartesianControl->movj(position);
    iCartesianControl->wait();

    // -- Position 6:
    printf("Position 6: poss (0 0 0 90 0 0 0)\n");
    position[0] = 0.390926; // 0.390926 -0.346663 0.166873 -0.004334 0.70944 0.704752 0.353119
    position[1] = -0.346663;
    position[2] = 0.166873;
    position[3] = -0.004334;
    position[4] = 0.70944;
    position[5] = 0.704752;
    position[6] = 0.353119;
    iCartesianControl->movj(position);
    iCartesianControl->wait();

    // -- Initial position
    printf("Position 7: Homing\n");
    position[0] = 3.25149848407618e-17;
    position[1] = -0.34692;
    position[2] = -0.221806;
    position[3] = 1.53080849893419e-16;
    position[4] = 1.0;
    position[5] = -3.06161699786838e-17;
    position[6] = 90.0;
    iCartesianControl->movj(position);
    iCartesianControl->wait();

    dd.close();

    return 0;
}

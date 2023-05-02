// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup kinematics-dynamics-examples
 * @defgroup cartesianControlExample cartesianControlExample
 *
 * <b>Running example with teoSim</b>
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * What mostly changes is the library command line invocation. We also change the server port name. The following is an example for the simulated robot's right arm.
\verbatim
[on terminal 2] teoSim
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/rightArm/CartesianControl --kinematics teo-fixedTrunk-rightArm-fetch.ini --local /BasicCartesianControl/teoSim/rightArm --remote /teoSim/rightArm
[on terminal 4] ./cartesianControlExample
[on possible terminal 5] yarp rpc /teoSim/rightArm/CartesianControl/rpc:s (for manual operation)
\endverbatim
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>

#include <ICartesianControl.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"cartesianRemote", yarp::os::Value("/teoSim/rightArm/CartesianControl")},
        {"cartesianLocal", yarp::os::Value("/CartesianControlExample")}
    };

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        yError() << "Device not available";
        return 1;
    }

    roboticslab::ICartesianControl *iCartesianControl;

    if (!dd.view(iCartesianControl))
    {
        yError() << "Problems acquiring interface";
        return 1;
    }

    std::vector<double> vector;
    iCartesianControl->stat(vector);

    yInfo() << "Controller status (forward kinematics):" << vector;

    yInfo() << "Position 1: poss (0 0 0 90 0 0)";

    if (!iCartesianControl->movj({0.4025, -0.3469, 0.1692, 0.0, 1.5708, 0.0}))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait();

    yInfo() << "Position 2: move forward along axis X";

    if (!iCartesianControl->movj({0.5, -0.3469, 0.1692, 0.0, 1.5708, 0.0}))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait();

    yInfo() << "Position 3: move right along axis Y";

    if (!iCartesianControl->movj({0.5, -0.4, 0.1692, 0.0, 1.5708, 0.0}))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait();

    yInfo() << "Position 4: rotate -12 degrees about global axis Y";

    if (!iCartesianControl->movj({0.5, -0.4, 0.1692, 0.0, 1.36, 0.0}))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait();

    yInfo() << "Position 5: rotate 45 degrees about global axis X";

    if (!iCartesianControl->movj({0.5, -0.4, 0.1692, 0.6139, 1.4822, 0.6139}))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait();

    yInfo() << "Position 6: poss (0 0 0 -90 0 0)";

    if (!iCartesianControl->movj({0.4025, -0.3469, 0.1692, 0.0, 1.5708, 0.0}))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait();

    yInfo() << "Position 7: homing";

    if (!iCartesianControl->movj({0.0, -0.3469, -0.2333, 0.0, 3.1416, 0.0}))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait();

    dd.close();
    return 0;
}

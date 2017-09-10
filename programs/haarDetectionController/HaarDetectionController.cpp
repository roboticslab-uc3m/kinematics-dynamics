// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetectionController.hpp"

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <ColorDebug.hpp>

using namespace roboticslab;

bool HaarDetectionController::configure(yarp::os::ResourceFinder &rf)
{
    std::string localPort = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PORT),
            "local cartesian port").asString();
    std::string remoteVision = rf.check("remoteVision", yarp::os::Value(DEFAULT_REMOTE_VISION),
            "remote vision port").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_REMOTE_CARTESIAN),
            "remote cartesian port").asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD), "period [s]").asDouble();

    yarp::os::Property cartesianControlClientOptions;
    cartesianControlClientOptions.put("device", "CartesianControlClient");
    cartesianControlClientOptions.put("cartesianLocal", localPort);
    cartesianControlClientOptions.put("cartesianRemote", remoteCartesian);

    if (!cartesianControlDevice.open(cartesianControlClientOptions))
    {
        CD_ERROR("Cartesian control client device not valid.\n");
        close();
        return false;
    }

    if (!cartesianControlDevice.view(iCartesianControl))
    {
        CD_ERROR("Could not view iCartesianControl.\n");
        close();
        return false;
    }

    grabberResponder.setICartesianControlDriver(iCartesianControl);

    grabberPort.useCallback(grabberResponder);
    grabberPort.open(localPort + "/state:i");

    if (!yarp::os::Network::connect(remoteVision + "/state:o", localPort + "/state:i"))
    {
        CD_ERROR("Unable to connect to remote vision port with prefix: %s.\n", remoteVision.c_str());
        close();
        return false;
    }

    return true;
}

bool HaarDetectionController::updateModule()
{
    return true;
}

bool HaarDetectionController::interruptModule()
{
    grabberPort.interrupt();

    if (iCartesianControl != NULL)
    {
        iCartesianControl->stopControl();
    }

    return true;
}

bool HaarDetectionController::close()
{
    grabberPort.close();
    return cartesianControlDevice.close();
}

double HaarDetectionController::getPeriod()
{
    return period;
}

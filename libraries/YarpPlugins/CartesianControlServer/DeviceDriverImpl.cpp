// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CartesianControlServer::open(yarp::os::Searchable& config)
{
    yarp::os::Value *name, *angleRepr;

    if (config.check("subdevice", name))
    {
        CD_INFO("Subdevice %s\n", name->toString().c_str());

        if (name->isString())
        {
            // maybe user isn't doing nested configuration
            yarp::os::Property p;
            p.fromString(config.toString());
            p.put("device", name->toString());
            p.setMonitor(config.getMonitor(), name->toString().c_str());
            cartesianControlDevice.open(p);
        }
        else
        {
            cartesianControlDevice.open(*name);
        }

        if (!cartesianControlDevice.isValid())
        {
            CD_ERROR("cannot make <%s>\n", name->toString().c_str());
        }
    }
    else
    {
        CD_ERROR("\"--subdevice <name>\" not set in CartesianControlServer\n");
        return false;
    }

    if (!cartesianControlDevice.isValid())
    {
        CD_ERROR("cartesianControlDevice not valid\n");
        return false;
    }

    if (!cartesianControlDevice.view(iCartesianControl))
    {
        CD_ERROR("iCartesianControl view failed\n");
        return false;
    }

    rpcResponder = new RpcResponder(iCartesianControl);
    streamResponder = new StreamResponder(iCartesianControl);

    std::string prefix = config.check("name", yarp::os::Value(DEFAULT_PREFIX), "local port prefix").asString();

    rpcServer.open(prefix + "/rpc:s");
    commandPort.open(prefix + "/command:i");

    rpcServer.setReader(*rpcResponder);
    commandPort.useCallback(*streamResponder);

    int periodInMs = config.check("fkPeriod", yarp::os::Value(DEFAULT_MS), "FK stream period (milliseconds)").asInt();

    if (periodInMs > 0)
    {
        fkOutPort.open(prefix + "/state:o");

        yarp::os::PeriodicThread::setPeriod(periodInMs * 0.001);
        yarp::os::PeriodicThread::start();
    }
    else
    {
        fkStreamEnabled = false;
    }

    // check angle representation, leave this block last to allow inner return instruction
    if (config.check("angleRepr", angleRepr, "angle representation for transform port"))
    {
        std::string angleReprStr = angleRepr->asString();
        KinRepresentation::orientation_system orient;

        if (!KinRepresentation::parseEnumerator(angleReprStr, &orient))
        {
            CD_WARNING("Unknown angleRepr \"%s\", falling back to default.\n", angleReprStr.c_str());
            return true;
        }

        rpcTransformResponder = new RpcTransformResponder(iCartesianControl, orient);

        rpcTransformServer.open(prefix + "/rpc_transform:s");
        rpcTransformServer.setReader(*rpcTransformResponder);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlServer::close()
{
    if (fkStreamEnabled)
    {
        yarp::os::PeriodicThread::stop();

        fkOutPort.interrupt();
        fkOutPort.close();
    }

    rpcServer.interrupt();
    rpcServer.close();
    delete rpcResponder;
    rpcResponder = NULL;

    if (rpcTransformResponder != NULL)
    {
        rpcTransformServer.interrupt();
        rpcTransformServer.close();
        delete rpcTransformResponder;
        rpcTransformResponder = NULL;
    }

    commandPort.interrupt();
    commandPort.close();
    delete streamResponder;
    streamResponder = NULL;

    return cartesianControlDevice.close();
}

// -----------------------------------------------------------------------------

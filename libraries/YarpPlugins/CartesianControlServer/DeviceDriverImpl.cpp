// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <string>

#include <ColorDebug.hpp>

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

    //Look for the portname to register (--name option)
    if (config.check("name", name))
    {
        rpcServer.open(name->asString() + "/rpc:s");
        commandPort.open(name->asString() + "/command:i");
        fkOutPort.open(name->asString() + "/state:o");
    }
    else
    {
        rpcServer.open("/CartesianControl/rpc:s");
        commandPort.open("/CartesianControl/command:i");
        fkOutPort.open("/CartesianControl/state:o");
    }

    rpcServer.setReader(*rpcResponder);
    commandPort.useCallback(*streamResponder);

    if (config.check("fkPeriod"))
    {
        int periodInMs = config.find("fkPeriod").asInt();
        yarp::os::RateThread::setRate(periodInMs);
    }

    yarp::os::RateThread::start();

    // check angle representation, leave this block last to allow inner return instruction
    if (config.check("angleRepr", angleRepr))
    {
        std::string angleReprStr = angleRepr->asString();
        KinRepresentation::orientation_system orient;

        if (!KinRepresentation::parseEnumerator(angleReprStr, &orient))
        {
            CD_WARNING("Unknown angleRepr \"%s\", falling back to default.\n", angleReprStr.c_str());
            return true;
        }

        rpcTransformResponder = new RpcTransformResponder(iCartesianControl, orient);

        if (config.check("name", name))
        {
            rpcTransformServer.open(name->asString() + "/rpc_transform:s");
        }
        else
        {
            rpcTransformServer.open("/CartesianControl/rpc_transform:s");
        }

        rpcTransformServer.setReader(*rpcTransformResponder);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlServer::close()
{
    yarp::os::RateThread::stop();

    fkOutPort.interrupt();
    fkOutPort.close();

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

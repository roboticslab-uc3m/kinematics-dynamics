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
    }
    else
    {
        rpcServer.open("/CartesianControl/rpc:s");
        commandPort.open("/CartesianControl/command:i");
    }

    // check angle representation, leave this block last to allow inner return instruction
    if (config.check("angleRepr", angleRepr))
    {
        std::string angleReprStr = angleRepr->asString();
        KinRepresentation::orientation_system orient;

        if (angleReprStr == "axisAngle")
        {
            orient = KinRepresentation::AXIS_ANGLE;
        }
        else if (angleReprStr == "eulerYZ")
        {
            orient = KinRepresentation::EULER_YZ;
        }
        else if (angleReprStr == "eulerZYZ")
        {
            orient = KinRepresentation::EULER_ZYZ;
        }
        else if (angleReprStr == "RPY")
        {
            orient = KinRepresentation::RPY;
        }
        else
        {
            CD_WARNING("Unknown angleRepr \"%s\".\n", angleReprStr.c_str());
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

    rpcServer.setReader(*rpcResponder);
    commandPort.useCallback(*streamResponder);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlServer::close()
{
    rpcServer.interrupt();
    rpcServer.close();
    delete rpcResponder;
    rpcResponder = NULL;

    rpcTransformServer.interrupt();
    rpcTransformServer.close();
    delete rpcTransformResponder;
    rpcTransformResponder = NULL;

    commandPort.interrupt();
    commandPort.close();
    delete streamResponder;
    streamResponder = NULL;

    cartesianControlDevice.close();

    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CartesianControlServer::open(yarp::os::Searchable& config)
{
    yarp::os::Value *name, *angleRepr, *coordRepr, *angularUnits;

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

    int periodInMs = config.check("fkPeriod", yarp::os::Value(DEFAULT_MS), "FK stream period (milliseconds)").asInt32();

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

    bool hasCoordRepr = config.check("coordRepr", coordRepr, "coordinate representation for transform port");
    bool hasAngleRepr = config.check("angleRepr", angleRepr, "angle representation for transform port");
    bool hasAngularUnits = config.check("angularUnits", angularUnits, "angular units for transform port");

    // check angle representation, leave this block last to allow inner return instruction
    if (hasCoordRepr || hasAngleRepr || hasAngularUnits)
    {
        std::string coordReprStr = coordRepr->asString();
        std::string angleReprStr = angleRepr->asString();
        std::string angularUnitsStr = angularUnits->asString();

        KinRepresentation::coordinate_system coord;
        KinRepresentation::orientation_system orient;
        KinRepresentation::angular_units units;

        if (!KinRepresentation::parseEnumerator(coordReprStr, &coord))
        {
            CD_WARNING("Unknown coordRepr \"%s\", falling back to default.\n", coordReprStr.c_str());
            return true;
        }

        if (!KinRepresentation::parseEnumerator(angleReprStr, &orient))
        {
            CD_WARNING("Unknown angleRepr \"%s\", falling back to default.\n", angleReprStr.c_str());
            return true;
        }

        if (!KinRepresentation::parseEnumerator(angularUnitsStr, &units))
        {
            CD_WARNING("Unknown angularUnits \"%s\", falling back to default.\n", angularUnitsStr.c_str());
            return true;
        }

        rpcTransformResponder = new RpcTransformResponder(iCartesianControl, coord, orient, units);

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

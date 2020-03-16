// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CartesianControlServer::open(yarp::os::Searchable& config)
{
    yarp::os::Value * name;

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

    bool ok = true;

    ok &= rpcServer.open(prefix + "/rpc:s");
    ok &= commandPort.open(prefix + "/command:i");

    rpcServer.setReader(*rpcResponder);
    commandPort.useCallback(*streamResponder);

    int periodInMs = config.check("fkPeriod", yarp::os::Value(DEFAULT_MS), "FK stream period (milliseconds)").asInt32();

    if (periodInMs > 0)
    {
        ok &= fkOutPort.open(prefix + "/state:o");

        yarp::os::PeriodicThread::setPeriod(periodInMs * 0.001);
        yarp::os::PeriodicThread::start();
    }
    else
    {
        fkStreamEnabled = false;
    }

    yarp::os::Value * angleRepr, * coordRepr, * angularUnits;

    KinRepresentation::coordinate_system coord = KinRepresentation::coordinate_system::CARTESIAN;
    KinRepresentation::orientation_system orient = KinRepresentation::orientation_system::AXIS_ANGLE_SCALED;
    KinRepresentation::angular_units units = KinRepresentation::angular_units::DEGREES;

    bool openTransformPort = false;

    if (config.check("coordRepr", coordRepr, "coordinate representation for transform port"))
    {
        std::string coordReprStr = coordRepr->asString();

        if (!KinRepresentation::parseEnumerator(coordReprStr, &coord))
        {
            CD_WARNING("Unknown coordRepr \"%s\", falling back to default.\n", coordReprStr.c_str());
        }
        else
        {
            openTransformPort = true;
        }
    }

    if (config.check("angleRepr", angleRepr, "angle representation for transform port"))
    {
        std::string angleReprStr = angleRepr->asString();

        if (!KinRepresentation::parseEnumerator(angleReprStr, &orient))
        {
            CD_WARNING("Unknown angleRepr \"%s\", falling back to default.\n", angleReprStr.c_str());
            return true;
        }
        else
        {
            openTransformPort = true;
        }
    }

    if (config.check("angularUnits", angularUnits, "angular units for transform port"))
    {
        std::string angularUnitsStr = angularUnits->asString();

        if (!KinRepresentation::parseEnumerator(angularUnitsStr, &units))
        {
            CD_WARNING("Unknown angularUnits \"%s\", falling back to default.\n", angularUnitsStr.c_str());
            return true;
        }
        else
        {
            openTransformPort = true;
        }
    }

    if (openTransformPort)
    {
        rpcTransformResponder = new RpcTransformResponder(iCartesianControl, coord, orient, units);
        ok &= rpcTransformServer.open(prefix + "/rpc_transform:s");
        rpcTransformServer.setReader(*rpcTransformResponder);
    }

    return ok;
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

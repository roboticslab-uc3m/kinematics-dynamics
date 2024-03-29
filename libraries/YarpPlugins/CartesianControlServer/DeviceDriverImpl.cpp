// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_PREFIX = "/CartesianServer";
constexpr auto DEFAULT_MS = 20;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlServer::open(yarp::os::Searchable& config)
{
    yarp::os::Value * name;

    if (config.check("subdevice", name))
    {
        yCInfo(CCS) << "Subdevice" << name->toString();

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
            yCError(CCS) << "Cannot make" << name->toString();
        }
    }
    else
    {
        yCError(CCS) << "Subdevice option not set in CartesianControlServer";
        return false;
    }

    if (!cartesianControlDevice.isValid())
    {
        yCError(CCS) << "Cartesian control device not valid";
        return false;
    }

    if (!cartesianControlDevice.view(iCartesianControl))
    {
        yCError(CCS) << "iCartesianControl view failed";
        return false;
    }

    rpcResponder = new RpcResponder(iCartesianControl);
    streamResponder = new StreamResponder(iCartesianControl);

    auto prefix = config.check("name", yarp::os::Value(DEFAULT_PREFIX), "local port prefix").asString();

    bool ok = true;

    ok &= rpcServer.open(prefix + "/rpc:s");
    ok &= commandPort.open(prefix + "/command:i");

    rpcServer.setReader(*rpcResponder);
    commandPort.useCallback(*streamResponder);

    int periodInMs = config.check("fkPeriod", yarp::os::Value(DEFAULT_MS), "FK stream period (milliseconds)").asInt32();

    if (periodInMs > 0)
    {
        fkStreamEnabled = true;
        ok &= fkOutPort.open(prefix + "/state:o");

        yarp::os::PeriodicThread::setPeriod(periodInMs * 0.001);
        yarp::os::PeriodicThread::start();
    }
    else
    {
        fkStreamEnabled = false;
    }

    yarp::os::Value * angleRepr, * coordRepr, * angularUnits;

    auto coord = KinRepresentation::coordinate_system::CARTESIAN;
    auto orient = KinRepresentation::orientation_system::AXIS_ANGLE_SCALED;
    auto units = KinRepresentation::angular_units::DEGREES;

    bool openTransformPort = false;

    if (config.check("coordRepr", coordRepr, "coordinate representation for transform port"))
    {
        auto coordReprStr = coordRepr->asString();

        if (!KinRepresentation::parseEnumerator(coordReprStr, &coord))
        {
            yCWarning(CCS, "Unknown coordRepr \"%s\", falling back to default", coordReprStr.c_str());
        }
        else
        {
            openTransformPort = true;
        }
    }

    if (config.check("angleRepr", angleRepr, "angle representation for transform port"))
    {
        auto angleReprStr = angleRepr->asString();

        if (!KinRepresentation::parseEnumerator(angleReprStr, &orient))
        {
            yCWarning(CCS, "Unknown angleRepr \"%s\", falling back to default", angleReprStr.c_str());
            return true;
        }
        else
        {
            openTransformPort = true;
        }
    }

    if (config.check("angularUnits", angularUnits, "angular units for transform port"))
    {
        auto angularUnitsStr = angularUnits->asString();

        if (!KinRepresentation::parseEnumerator(angularUnitsStr, &units))
        {
            yCWarning(CCS, "Unknown angularUnits \"%s\", falling back to default", angularUnitsStr.c_str());
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

bool CartesianControlServer::close()
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
    rpcResponder = nullptr;

    if (rpcTransformResponder)
    {
        rpcTransformServer.interrupt();
        rpcTransformServer.close();
        delete rpcTransformResponder;
        rpcTransformResponder = nullptr;
    }

    commandPort.interrupt();
    commandPort.close();
    delete streamResponder;
    streamResponder = nullptr;

    return cartesianControlDevice.close();
}

// -----------------------------------------------------------------------------

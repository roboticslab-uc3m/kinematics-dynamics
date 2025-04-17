// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlServer::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(CCS) << "Failed to parse parameters";
        return false;
    }

    if (!rpcServer.open(m_name + "/rpc:s"))
    {
        yCError(CCS) << "Failed to open RPC port";
        return false;
    }

    if (!commandPort.open(m_name + "/command:i"))
    {
        yCError(CCS) << "Failed to open command port";
        return false;
    }

    rpcResponder = new RpcResponder();
    streamResponder = new StreamResponder();

    rpcServer.setReader(*rpcResponder);
    commandPort.useCallback(*streamResponder);

    if (m_fkPeriod > 0)
    {
        yarp::os::PeriodicThread::setPeriod(m_fkPeriod * 0.001);

        if (!fkOutPort.open(m_name + "/state:o"))
        {
            yCError(CCS) << "Failed to open FK stream port";
            return false;
        }
    }

    KinRepresentation::coordinate_system coord;
    KinRepresentation::orientation_system orient;
    KinRepresentation::angular_units units;

    // keep defaults in sync with CartesianControlServer_ParamsParser
    KinRepresentation::parseEnumerator(m_coordRepr_defaultValue, &coord, KinRepresentation::coordinate_system::CARTESIAN);
    KinRepresentation::parseEnumerator(m_angleRepr_defaultValue, &orient, KinRepresentation::orientation_system::AXIS_ANGLE_SCALED);
    KinRepresentation::parseEnumerator(m_angularUnits_defaultValue, &units, KinRepresentation::angular_units::DEGREES);

    bool openTransformPort = false;

    if (m_coordRepr != m_coordRepr_defaultValue)
    {
        if (!KinRepresentation::parseEnumerator(m_coordRepr, &coord))
        {
            yCWarning(CCS, "Unknown coordRepr \"%s\", falling back to default", m_coordRepr.c_str());
        }
        else
        {
            openTransformPort = true;
        }
    }

    if (m_angleRepr != m_angleRepr_defaultValue)
    {
        if (!KinRepresentation::parseEnumerator(m_angleRepr, &orient))
        {
            yCWarning(CCS, "Unknown angleRepr \"%s\", falling back to default", m_angleRepr.c_str());
        }
        else
        {
            openTransformPort = true;
        }
    }

    if (m_angularUnits != m_angularUnits_defaultValue)
    {
        if (!KinRepresentation::parseEnumerator(m_angularUnits, &units, KinRepresentation::angular_units::DEGREES))
        {
            yCWarning(CCS, "Unknown angularUnits \"%s\", falling back to default", m_angularUnits.c_str());
        }
        else
        {
            openTransformPort = true;
        }
    }

    if (openTransformPort && coord == KinRepresentation::coordinate_system::NONE && orient == KinRepresentation::orientation_system::NONE)
    {
        yCWarning(CCS) << "Transform port not opened, both coordinate and orientation systems are set as NONE";
        openTransformPort = false;
    }

    if (openTransformPort)
    {
        rpcTransformResponder = new RpcTransformResponder(coord, orient, units);
        rpcTransformServer.setReader(*rpcTransformResponder);

        if (!rpcTransformServer.open(m_name + "/rpc_transform:s"))
        {
            yCError(CCS) << "Failed to open transform RPC port";
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlServer::close()
{
    if (!fkOutPort.isClosed())
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

    return true;
}

// -----------------------------------------------------------------------------

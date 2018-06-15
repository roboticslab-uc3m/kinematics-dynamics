// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TwoLimbCartesianControlServer.hpp"

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::TwoLimbCartesianControlServer::open(yarp::os::Searchable& config) {

    rpcServer.setReader(*this);

    yarp::os::Value *name;
    if (config.check("subdevice",name))
    {
        CD_INFO("Subdevice %s\n", name->toString().c_str());
        if (name->isString())
        {
            // maybe user isn't doing nested configuration
            yarp::os::Property p;
            p.fromString(config.toString());
            p.put("device",name->toString());
            cartesianControlDevice.open(p);
        }
        else
            cartesianControlDevice.open(*name);
        if (!cartesianControlDevice.isValid())
            CD_ERROR("cannot make <%s>\n", name->toString().c_str());
    }
    else
    {
        CD_ERROR("\"--subdevice <name>\" not set in TwoLimbCartesianControlServer\n");
        return false;
    }
    if( ! cartesianControlDevice.isValid() )
    {
        CD_ERROR("cartesianControlDevice not valid\n");
        return false;
    }
    if( ! cartesianControlDevice.view( iTwoLimbCartesianControl ) )
    {
        CD_ERROR("iTwoLimbCartesianControl view failed\n");
        return false;
    }

    //Look for the portname to register (--name option)
    if (config.check("name",name))
        rpcServer.open(name->asString()+"/rpc:s");
    else
        rpcServer.open("/TwoLimbCartesianControl/rpc:s");


    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TwoLimbCartesianControlServer::close()
{
    rpcServer.close();
    cartesianControlDevice.close();
    return true;
}

// -----------------------------------------------------------------------------

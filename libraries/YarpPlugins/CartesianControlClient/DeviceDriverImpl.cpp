// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CartesianControlClient::open(yarp::os::Searchable& config)
{
    std::string local = config.check("cartesianLocal", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),
            "local port").asString();
    std::string remote = config.check("cartesianRemote", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
            "remote port").asString();

    bool portsOk = true;

    portsOk = portsOk && rpcClient.open(local + "/rpc:c");
    portsOk = portsOk && commandPort.open(local + "/command:o");
    portsOk = portsOk && fkInPort.open(local + "/state:i");

    if (!portsOk)
    {
        CD_ERROR("Unable to open ports.\n");
        return false;
    }

    std::string suffix = config.check("transform", "connect to server transform port") ? "/rpc_transform:s" : "/rpc:s";

    if (!rpcClient.addOutput(remote + suffix))
    {
        CD_ERROR("Error on connect to remote RPC server.\n");
        close();  // close ports
        return false;
    }

    if (!commandPort.addOutput(remote + "/command:i", "udp"))
    {
        CD_ERROR("Error on connect to remote command server.\n");
        close();  // close ports
        return false;
    }

    fkStreamTimeoutSecs = config.check("fkStreamTimeoutSecs", yarp::os::Value(DEFAULT_FK_STREAM_TIMEOUT_SECS),
            "FK stream timeout (seconds)").asDouble();

    if (!yarp::os::Network::connect(remote + "/state:o", fkInPort.getName(), "udp"))
    {
        CD_WARNING("FK stream disabled, using RPC instead.\n");
        fkStreamEnabled = false;
        fkInPort.close();
    }
    else
    {
        fkStreamEnabled = true;
        fkInPort.useCallback(fkStreamResponder);
        yarp::os::Time::delay(fkStreamTimeoutSecs); // wait for first data to arrive
    }

    CD_SUCCESS("Connected to remote.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::close()
{
    rpcClient.close();
    commandPort.close();

    if (fkStreamEnabled)
    {
        fkInPort.close();
    }

    return true;
}

// -----------------------------------------------------------------------------

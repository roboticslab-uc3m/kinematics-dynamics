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
            "cartesianLocal").asString();
    std::string remote = config.check("cartesianRemote", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
            "cartesianRemote").asString();

    bool portsOk = true;

    portsOk = portsOk && rpcClient.open(local + "/rpc:c");
    portsOk = portsOk && commandPort.open(local + "/command:o");
    portsOk = portsOk && fkInPort.open(local + "/state:i");

    if (!portsOk)
    {
        CD_ERROR("Unable to open ports.\n");
        return false;
    }

    int tries = 0;
    const int maxTries = 10;
    const double waitInSeconds = 0.5;

    while (tries++ < maxTries)
    {
        std::string suffix = config.check("transform") ? "/rpc_transform:s" : "/rpc:s";

        if (rpcClient.addOutput(remote + suffix))
        {
            break;
        }

        CD_DEBUG("Wait to connect to remote RPC server, try %d...\n", tries);
        yarp::os::Time::delay(waitInSeconds);
    }

    if (tries > maxTries)
    {
        CD_ERROR("Timeout on connect to remote RPC server!\n");
        close();  // close ports
        return false;
    }

    tries = 0;

    while (tries++ < maxTries)
    {
        if (commandPort.addOutput(remote + "/command:i", "udp"))
        {
            break;
        }

        CD_DEBUG("Wait to connect to remote command server, try %d...\n", tries);
        yarp::os::Time::delay(waitInSeconds);
    }

    if (tries > maxTries)
    {
        CD_ERROR("Timeout on connect to remote command server!\n");
        close();  // close ports
        return false;
    }

    tries = 0;

    while (tries++ < maxTries)
    {
        if (yarp::os::Network::connect(remote + "/state:o", fkInPort.getName(), "udp"))
        {
            break;
        }

        CD_DEBUG("Wait to connect to remote FK stream server, try %d...\n", tries);
        yarp::os::Time::delay(waitInSeconds);
    }

    if (tries > maxTries)
    {
        CD_ERROR("Timeout on connect to remote FK stream server!\n");
        close();  // close ports
        return false;
    }

    CD_SUCCESS("Connected to remote.\n");

    fkInPort.useCallback(fkStreamResponder);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::close()
{
    rpcClient.close();
    commandPort.close();
    fkInPort.close();

    return true;
}

// -----------------------------------------------------------------------------

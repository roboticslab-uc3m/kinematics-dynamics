// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <string>

#include <yarp/os/Time.h>

#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CartesianControlClient::open(yarp::os::Searchable& config)
{
    std::string local = config.check("cartesianLocal", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),
            "cartesianLocal").asString();
    std::string remote = config.check("cartesianRemote", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
            "cartesianRemote").asString();

    rpcClient.open(local + "/rpc:c");
    commandPort.open(local + "/command:o");

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
        rpcClient.close();
        commandPort.close();
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
        rpcClient.close();
        commandPort.close();
        return false;
    }

    CD_SUCCESS("Connected to remote.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::close()
{
    rpcClient.close();
    commandPort.close();
    return true;
}

// -----------------------------------------------------------------------------

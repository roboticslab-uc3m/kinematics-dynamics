// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CartesianControlClient::open(yarp::os::Searchable& config) {

    std::string local = config.check("cartesianLocal",yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),"cartesianLocal").asString();
    std::string remote = config.check("cartesianRemote",yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),"cartesianRemote").asString();

    rpcClient.open(local + "/rpc:c");
    commandPort.open(local + "/command:o");

    int tries = 0;

    while (tries++ < 10)
    {
        yarp::os::Network::connect(local + "/rpc:c", remote + "/rpc:s");
        if (rpcClient.getOutputCount() > 0) break;
        CD_DEBUG("Wait to connect to remote RPC server, try %d...\n", tries);
        yarp::os::Time::delay(0.5);
    }

    if (tries == 11)
    {
        CD_ERROR("Timeout on connect to remote RPC server!\n");
        rpcClient.close();
        commandPort.close();
        return false;
    }

    tries = 0;

    while (tries++ < 10)
    {
        yarp::os::Network::connect(local + "/command:o", remote + "/command:i");
        if (commandPort.getOutputCount() > 0) break;
        CD_DEBUG("Wait to connect to remote command server, try %d...\n", tries);
        yarp::os::Time::delay(0.5);
    }

    if (tries == 11)
    {
        CD_ERROR("Timeout on connect to remote command server!\n");
        rpcClient.close();
        commandPort.close();
        return false;
    }

    CD_SUCCESS("Connected to remote.\n");

    commandBuffer.attach(commandPort);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::close()
{
    rpcClient.close();
    commandBuffer.detach();
    commandPort.close();
    return true;
}

// -----------------------------------------------------------------------------

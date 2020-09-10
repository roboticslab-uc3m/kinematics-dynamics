// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CartesianControlClient::open(yarp::os::Searchable& config)
{
    std::string local = config.check("cartesianLocal", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),
            "local port").asString();
    std::string remote = config.check("cartesianRemote", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
            "remote port").asString();

    if (!rpcClient.open(local + "/rpc:c") || !commandPort.open(local + "/command:o"))
    {
        CD_ERROR("Unable to open ports.\n");
        return false;
    }

    bool transformEnabled = config.check("transform", "connect to server transform port");
    std::string suffix = transformEnabled ? "/rpc_transform:s" : "/rpc:s";

    if (!rpcClient.addOutput(remote + suffix))
    {
        CD_ERROR("Error on connect to remote RPC server.\n");
        return false;
    }

    if (!commandPort.addOutput(remote + "/command:i", "udp"))
    {
        CD_ERROR("Error on connect to remote command server.\n");
        return false;
    }

    fkStreamTimeoutSecs = config.check("fkStreamTimeoutSecs", yarp::os::Value(DEFAULT_FK_STREAM_TIMEOUT_SECS),
            "FK stream timeout (seconds)").asFloat64();

    if (transformEnabled)
    {
        // Incoming FK stream data may not conform to standard representation, resort to RPC
        // if user requests --transform (see #143, #145).
        CD_WARNING("FK streaming not supported in --transform mode, using RPC instead.\n");
    }
    else
    {
        std::string statePort = remote + "/state:o";

        if (yarp::os::Network::exists(statePort))
        {
            if (!fkInPort.open(local + "/state:i"))
            {
                CD_ERROR("Unable to open local stream port.\n");
                return false;
            }

            if (!yarp::os::Network::connect(statePort, fkInPort.getName(), "udp"))
            {
                CD_ERROR("Unable to connect to remote stream port.\n");
                return false;
            }

            fkInPort.useCallback(fkStreamResponder);
            yarp::os::Time::delay(fkStreamTimeoutSecs); // wait for first data to arrive
        }
        else
        {
            CD_WARNING("Missing remote %s stream port, using RPC instead.\n", statePort.c_str());
        }
    }

    CD_SUCCESS("Connected to remote.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::close()
{
    rpcClient.close();
    commandPort.close();

    if (!fkInPort.isClosed())
    {
        fkInPort.close();
    }

    return true;
}

// -----------------------------------------------------------------------------

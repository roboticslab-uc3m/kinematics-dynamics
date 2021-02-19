// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CartesianControlClient::open(yarp::os::Searchable& config)
{
    std::string local = config.check("cartesianLocal", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),
            "local port").asString();
    std::string remote = config.check("cartesianRemote", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
            "remote port").asString();

    if (!rpcClient.open(local + "/rpc:c") || !commandPort.open(local + "/command:o"))
    {
        yError() << "Unable to open ports";
        return false;
    }

    bool transformEnabled = config.check("transform", "connect to server transform port");
    std::string suffix = transformEnabled ? "/rpc_transform:s" : "/rpc:s";

    if (!rpcClient.addOutput(remote + suffix))
    {
        yError() << "Error on connect to remote RPC server";
        return false;
    }

    if (!commandPort.addOutput(remote + "/command:i", "udp"))
    {
        yError() << "Error on connect to remote command server";
        return false;
    }

    fkStreamTimeoutSecs = config.check("fkStreamTimeoutSecs", yarp::os::Value(DEFAULT_FK_STREAM_TIMEOUT_SECS),
            "FK stream timeout (seconds)").asFloat64();

    if (transformEnabled)
    {
        // Incoming FK stream data may not conform to standard representation, resort to RPC
        // if user requests --transform (see #143, #145).
        yWarning() << "FK streaming not supported in --transform mode, using RPC instead";
    }
    else
    {
        std::string statePort = remote + "/state:o";

        if (yarp::os::Network::exists(statePort))
        {
            if (!fkInPort.open(local + "/state:i"))
            {
                yError() << "Unable to open local stream port";
                return false;
            }

            if (!yarp::os::Network::connect(statePort, fkInPort.getName(), "udp"))
            {
                yError() << "Unable to connect to remote stream port";
                return false;
            }

            fkInPort.useCallback(fkStreamResponder);
            yarp::os::Time::delay(fkStreamTimeoutSecs); // wait for first data to arrive
        }
        else
        {
            yWarning() << "Missing remote" << statePort << "stream port, using RPC instead";
        }
    }

    yInfo() << "Connected to remote";

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

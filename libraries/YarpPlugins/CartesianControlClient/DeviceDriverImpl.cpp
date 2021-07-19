// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_CARTESIAN_LOCAL = "/CartesianControlClient";
constexpr auto DEFAULT_CARTESIAN_REMOTE = "/CartesianControl";
constexpr auto DEFAULT_FK_STREAM_TIMEOUT_SECS = 0.5;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlClient::open(yarp::os::Searchable& config)
{
    auto local = config.check("cartesianLocal", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL),
            "local port").asString();
    auto remote = config.check("cartesianRemote", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE),
            "remote port").asString();

    if (!rpcClient.open(local + "/rpc:c") || !commandPort.open(local + "/command:o"))
    {
        yCError(CCC) << "Unable to open ports";
        return false;
    }

    if (!rpcClient.addOutput(remote + "/rpc:s"))
    {
        yCError(CCC) << "Error on connect to remote RPC server";
        return false;
    }

    if (!commandPort.addOutput(remote + "/command:i", "udp"))
    {
        yCError(CCC) << "Error on connect to remote command server";
        return false;
    }

    fkStreamTimeoutSecs = config.check("fkStreamTimeoutSecs", yarp::os::Value(DEFAULT_FK_STREAM_TIMEOUT_SECS),
            "FK stream timeout (seconds)").asFloat64();

    std::string statePort = remote + "/state:o";

    if (yarp::os::Network::exists(statePort))
    {
        if (!fkInPort.open(local + "/state:i"))
        {
            yCError(CCC) << "Unable to open local stream port";
            return false;
        }

        if (!yarp::os::Network::connect(statePort, fkInPort.getName(), "udp"))
        {
            yCError(CCC) << "Unable to connect to remote stream port";
            return false;
        }

        fkInPort.useCallback(fkStreamResponder);
        yarp::os::Time::delay(fkStreamTimeoutSecs); // wait for first data to arrive
    }
    else
    {
        yCWarning(CCC) << "Missing remote" << statePort << "stream port, using RPC instead";
    }

    yCInfo(CCC) << "Connected to remote" << remote;

    return true;
}

// -----------------------------------------------------------------------------

bool CartesianControlClient::close()
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

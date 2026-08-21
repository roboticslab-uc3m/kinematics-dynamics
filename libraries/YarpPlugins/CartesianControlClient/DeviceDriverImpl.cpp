// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlClient::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(CCC) << "Failed to parse parameters";
        return false;
    }

    if (!rpcClient.open(m_local + "/rpc:c") || !commandPort.open(m_local + "/command:o"))
    {
        yCError(CCC) << "Unable to open local RPC and command ports";
        return false;
    }

    if (!rpcClient.addOutput(m_remote + "/rpc:s"))
    {
        yCError(CCC) << "Error on connect to remote RPC server";
        return false;
    }

    if (!commandPort.addOutput(m_remote + "/command:i", "fast_tcp"))
    {
        yCError(CCC) << "Error on connect to remote command server";
        return false;
    }

    if (!rpcSender.yarp().attachAsClient(rpcClient))
    {
        yCError(CCC) << "Unable to attach RPC sender to port";
        return false;
    }

    std::string statePort = m_remote + "/state:o";

    if (yarp::os::Network::exists(statePort))
    {
        if (!fkInPort.open(m_local + "/state:i"))
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
        yarp::os::SystemClock::delaySystem(m_fkStreamTimeoutSecs); // wait for first data to arrive
    }
    else
    {
        yCWarning(CCC) << "Missing remote" << statePort << "stream port, using RPC instead";
    }

    yCInfo(CCC) << "Connected to remote" << m_remote;

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

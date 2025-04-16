// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

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

    if (!rpcClient.open(m_cartesianLocal+ "/rpc:c") || !commandPort.open(m_cartesianLocal + "/command:o"))
    {
        yCError(CCC) << "Unable to open local RPC and command ports";
        return false;
    }

    if (!rpcClient.addOutput(m_cartesianRemote + "/rpc:s"))
    {
        yCError(CCC) << "Error on connect to remote RPC server";
        return false;
    }

    if (!commandPort.addOutput(m_cartesianRemote + "/command:i", "udp"))
    {
        yCError(CCC) << "Error on connect to remote command server";
        return false;
    }

    std::string statePort = m_cartesianRemote + "/state:o";

    if (yarp::os::Network::exists(statePort))
    {
        if (!fkInPort.open(m_cartesianLocal + "/state:i"))
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
        yarp::os::Time::delay(m_fkStreamTimeoutSecs); // wait for first data to arrive
    }
    else
    {
        yCWarning(CCC) << "Missing remote" << statePort << "stream port, using RPC instead";
    }

    yCInfo(CCC) << "Connected to remote" << m_cartesianRemote;

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

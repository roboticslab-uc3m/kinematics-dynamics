/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Wed Apr 16 14:50:07 2025


#include "CartesianControlClient_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(CartesianControlClientParamsCOMPONENT, "yarp.device.CartesianControlClient")
}


CartesianControlClient_ParamsParser::CartesianControlClient_ParamsParser()
{
}


std::vector<std::string> CartesianControlClient_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("cartesianLocal");
    params.push_back("cartesianRemote");
    params.push_back("fkStreamTimeoutSecs");
    return params;
}


bool      CartesianControlClient_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(CartesianControlClientParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    std::string config_string = config.toString();
    yarp::os::Property prop_check(config_string.c_str());
    //Parser of parameter cartesianLocal
    {
        if (config.check("cartesianLocal"))
        {
            m_cartesianLocal = config.find("cartesianLocal").asString();
            yCInfo(CartesianControlClientParamsCOMPONENT) << "Parameter 'cartesianLocal' using value:" << m_cartesianLocal;
        }
        else
        {
            yCInfo(CartesianControlClientParamsCOMPONENT) << "Parameter 'cartesianLocal' using DEFAULT value:" << m_cartesianLocal;
        }
        prop_check.unput("cartesianLocal");
    }

    //Parser of parameter cartesianRemote
    {
        if (config.check("cartesianRemote"))
        {
            m_cartesianRemote = config.find("cartesianRemote").asString();
            yCInfo(CartesianControlClientParamsCOMPONENT) << "Parameter 'cartesianRemote' using value:" << m_cartesianRemote;
        }
        else
        {
            yCInfo(CartesianControlClientParamsCOMPONENT) << "Parameter 'cartesianRemote' using DEFAULT value:" << m_cartesianRemote;
        }
        prop_check.unput("cartesianRemote");
    }

    //Parser of parameter fkStreamTimeoutSecs
    {
        if (config.check("fkStreamTimeoutSecs"))
        {
            m_fkStreamTimeoutSecs = config.find("fkStreamTimeoutSecs").asFloat64();
            yCInfo(CartesianControlClientParamsCOMPONENT) << "Parameter 'fkStreamTimeoutSecs' using value:" << m_fkStreamTimeoutSecs;
        }
        else
        {
            yCInfo(CartesianControlClientParamsCOMPONENT) << "Parameter 'fkStreamTimeoutSecs' using DEFAULT value:" << m_fkStreamTimeoutSecs;
        }
        prop_check.unput("fkStreamTimeoutSecs");
    }

    /*
    //This code check if the user set some parameter which are not check by the parser
    //If the parser is set in strict mode, this will generate an error
    if (prop_check.size() > 0)
    {
        bool extra_params_found = false;
        for (auto it=prop_check.begin(); it!=prop_check.end(); it++)
        {
            if (m_parser_is_strict)
            {
                yCError(CartesianControlClientParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(CartesianControlClientParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
            }
        }

       if (m_parser_is_strict && extra_params_found)
       {
           return false;
       }
    }
    */
    return true;
}


std::string      CartesianControlClient_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: CartesianControlClient\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'cartesianLocal': local port\n");
    doc = doc + std::string("'cartesianRemote': remote port\n");
    doc = doc + std::string("'fkStreamTimeoutSecs': FK stream timeout\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device CartesianControlClient --cartesianLocal /CartesianControlClient --cartesianRemote /CartesianControl --fkStreamTimeoutSecs 0.5\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device CartesianControlClient\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}

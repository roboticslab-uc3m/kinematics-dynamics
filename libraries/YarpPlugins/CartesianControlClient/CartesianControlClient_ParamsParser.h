/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Wed Apr 16 14:50:07 2025


#ifndef CARTESIANCONTROLCLIENT_PARAMSPARSER_H
#define CARTESIANCONTROLCLIENT_PARAMSPARSER_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/IDeviceDriverParams.h>
#include <string>
#include <cmath>

/**
* This class is the parameters parser for class CartesianControlClient.
*
* These are the used parameters:
* | Group name | Parameter name      | Type   | Units | Default Value           | Required | Description       | Notes |
* |:----------:|:-------------------:|:------:|:-----:|:-----------------------:|:--------:|:-----------------:|:-----:|
* | -          | cartesianLocal      | string | -     | /CartesianControlClient | 0        | local port        | -     |
* | -          | cartesianRemote     | string | -     | /CartesianControl       | 0        | remote port       | -     |
* | -          | fkStreamTimeoutSecs | double | s     | 0.5                     | 0        | FK stream timeout | -     |
*
* The device can be launched by yarpdev using one of the following examples (with and without all optional parameters):
* \code{.unparsed}
* yarpdev --device CartesianControlClient --cartesianLocal /CartesianControlClient --cartesianRemote /CartesianControl --fkStreamTimeoutSecs 0.5
* \endcode
*
* \code{.unparsed}
* yarpdev --device CartesianControlClient
* \endcode
*
*/

class CartesianControlClient_ParamsParser : public yarp::dev::IDeviceDriverParams
{
public:
    CartesianControlClient_ParamsParser();
    ~CartesianControlClient_ParamsParser() override = default;

public:
    const std::string m_device_classname = {"CartesianControlClient"};
    const std::string m_device_name = {"CartesianControlClient"};
    bool m_parser_is_strict = false;
    struct parser_version_type
    {
         int major = 1;
         int minor = 0;
    };
    const parser_version_type m_parser_version = {};

    const std::string m_cartesianLocal_defaultValue = {"/CartesianControlClient"};
    const std::string m_cartesianRemote_defaultValue = {"/CartesianControl"};
    const std::string m_fkStreamTimeoutSecs_defaultValue = {"0.5"};

    std::string m_cartesianLocal = {"/CartesianControlClient"};
    std::string m_cartesianRemote = {"/CartesianControl"};
    double m_fkStreamTimeoutSecs = {0.5};

    bool          parseParams(const yarp::os::Searchable & config) override;
    std::string   getDeviceClassName() const override { return m_device_classname; }
    std::string   getDeviceName() const override { return m_device_name; }
    std::string   getDocumentationOfDeviceParams() const override;
    std::vector<std::string> getListOfParams() const override;
};

#endif

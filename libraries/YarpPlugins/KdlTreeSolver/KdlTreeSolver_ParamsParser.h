/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Wed Apr 16 13:53:33 2025


#ifndef KDLTREESOLVER_PARAMSPARSER_H
#define KDLTREESOLVER_PARAMSPARSER_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/IDeviceDriverParams.h>
#include <string>
#include <cmath>

/**
* This class is the parameters parser for class KdlTreeSolver.
*
* These are the used parameters:
* | Group name | Parameter name | Type           | Units | Default Value  | Required | Description                                       | Notes        |
* |:----------:|:--------------:|:--------------:|:-----:|:--------------:|:--------:|:-------------------------------------------------:|:------------:|
* | -          | kinematics     | string         | -     | -              | 0        | path to file with description of robot kinematics | -            |
* | -          | gravity        | vector<double> | m/s^2 | (0.0 0.0 9.81) | 0        | gravity vector                                    | -            |
* | -          | ikPos          | string         | -     | nrjl           | 0        | IK position solver algorithm                      | nrjl, online |
* | -          | epsPos         | double         | m     | 1e-5           | 0        | IK position solver precision                      | -            |
* | -          | maxIterPos     | int            | -     | 1000           | 0        | IK position solver max iterations                 | -            |
* | -          | vTranslMax     | double         | m/s   | 1.0            | 0        | maximum translation speed                         | -            |
* | -          | vRotMax        | double         | deg/s | 50.0           | 0        | maximum rotation speed                            | -            |
* | -          | lambda         | double         | -     | 0.01           | 0        | lambda parameter for diff IK                      | -            |
* | -          | weightsJS      | vector<double> | -     | (0.0)          | 0        | joint space weights                               | -            |
* | -          | weightsTS      | vector<double> | -     | (0.0)          | 0        | task space weights                                | -            |
* | -          | mins           | vector<double> | deg   | (0.0)          | 0        | lower bound joint position limits                 | -            |
* | -          | maxs           | vector<double> | deg   | (0.0)          | 0        | upper bound joint position limits                 | -            |
* | -          | maxvels        | vector<double> | deg/s | (0.0)          | 0        | joint velocity limits                             | -            |
*
* The device can be launched by yarpdev using one of the following examples (with and without all optional parameters):
* \code{.unparsed}
* yarpdev --device KdlTreeSolver --kinematics <optional_value> --gravity \" (0.0 0.0 9.81) \" --ikPos nrjl --epsPos 1e-5 --maxIterPos 1000 --vTranslMax 1.0 --vRotMax 50.0 --lambda 0.01 --weightsJS \" (0.0) \" --weightsTS \" (0.0) \" --mins \" (0.0) \" --maxs \" (0.0) \" --maxvels \" (0.0) \"
* \endcode
*
* \code{.unparsed}
* yarpdev --device KdlTreeSolver
* \endcode
*
*/

class KdlTreeSolver_ParamsParser : public yarp::dev::IDeviceDriverParams
{
public:
    KdlTreeSolver_ParamsParser();
    ~KdlTreeSolver_ParamsParser() override = default;

public:
    const std::string m_device_classname = {"KdlTreeSolver"};
    const std::string m_device_name = {"KdlTreeSolver"};
    bool m_parser_is_strict = false;
    struct parser_version_type
    {
         int major = 1;
         int minor = 0;
    };
    const parser_version_type m_parser_version = {};

    const std::string m_kinematics_defaultValue = {""};
    const std::string m_gravity_defaultValue = {"(0.0 0.0 9.81)"};
    const std::string m_ikPos_defaultValue = {"nrjl"};
    const std::string m_epsPos_defaultValue = {"1e-5"};
    const std::string m_maxIterPos_defaultValue = {"1000"};
    const std::string m_vTranslMax_defaultValue = {"1.0"};
    const std::string m_vRotMax_defaultValue = {"50.0"};
    const std::string m_lambda_defaultValue = {"0.01"};
    const std::string m_weightsJS_defaultValue = {"(0.0)"};
    const std::string m_weightsTS_defaultValue = {"(0.0)"};
    const std::string m_mins_defaultValue = {"(0.0)"};
    const std::string m_maxs_defaultValue = {"(0.0)"};
    const std::string m_maxvels_defaultValue = {"(0.0)"};

    std::string m_kinematics = {}; //This default value of this string is an empty string. It is highly recommended to provide a suggested value also for optional string parameters.
    std::vector<double> m_gravity = { }; //Default values for lists are managed in the class constructor. It is highly recommended to provide a suggested value also for optional string parameters.
    std::string m_ikPos = {"nrjl"};
    double m_epsPos = {1e-5};
    int m_maxIterPos = {1000};
    double m_vTranslMax = {1.0};
    double m_vRotMax = {50.0};
    double m_lambda = {0.01};
    std::vector<double> m_weightsJS = { }; //Default values for lists are managed in the class constructor. It is highly recommended to provide a suggested value also for optional string parameters.
    std::vector<double> m_weightsTS = { }; //Default values for lists are managed in the class constructor. It is highly recommended to provide a suggested value also for optional string parameters.
    std::vector<double> m_mins = { }; //Default values for lists are managed in the class constructor. It is highly recommended to provide a suggested value also for optional string parameters.
    std::vector<double> m_maxs = { }; //Default values for lists are managed in the class constructor. It is highly recommended to provide a suggested value also for optional string parameters.
    std::vector<double> m_maxvels = { }; //Default values for lists are managed in the class constructor. It is highly recommended to provide a suggested value also for optional string parameters.

    bool          parseParams(const yarp::os::Searchable & config) override;
    std::string   getDeviceClassName() const override { return m_device_classname; }
    std::string   getDeviceName() const override { return m_device_name; }
    std::string   getDocumentationOfDeviceParams() const override;
    std::vector<std::string> getListOfParams() const override;
};

#endif

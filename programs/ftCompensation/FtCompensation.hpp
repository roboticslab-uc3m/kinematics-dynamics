// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FT_COMPENSATION_HPP__
#define __FT_COMPENSATION_HPP__

#include <yarp/os/RFModule.h>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include "ICartesianControl.h"

namespace roboticslab
{

/**
 * @ingroup ftCompensation
 *
 * @brief ...
 */
class FtCompensation : public yarp::os::RFModule
{
public:
    ~FtCompensation() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    double getPeriod() override;
    bool close() override;

private:
    bool compensateTool(KDL::Wrench & sensorWrench) const;

    yarp::dev::PolyDriver cartesianDevice;
    roboticslab::ICartesianControl * iCartesianControl;

    int sensorIndex;
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::ISixAxisForceTorqueSensors * sensor;

    KDL::Rotation R_N_sensor;
    KDL::Vector toolCoM_N;
    KDL::Wrench toolWeight_0;
    KDL::Wrench initialOffset;

    bool usingTool;
    double linGain, rotGain;
    double linDeadband, rotDeadband;
};

} // namespace roboticslab

#endif // __FT_COMPENSATION_HPP__

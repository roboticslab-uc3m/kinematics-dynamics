// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FT_COMPENSATION_HPP__
#define __FT_COMPENSATION_HPP__

#include <yarp/conf/version.h>

#include <yarp/os/PeriodicThread.h>
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
class FtCompensation : public yarp::os::RFModule,
                       public yarp::os::PeriodicThread
{
public:
    FtCompensation()
#if YARP_VERSION_MINOR >= 5
        : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes, yarp::os::PeriodicThreadClock::Absolute)
#else
        : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes)
#endif
    {}

    ~FtCompensation() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    double getPeriod() override;
    bool close() override;

protected:
    void run() override;

private:
    bool readSensor(KDL::Wrench & wrench) const;
    bool compensateTool(KDL::Wrench & wrench) const;

    yarp::dev::PolyDriver cartesianDevice;
    roboticslab::ICartesianControl * iCartesianControl;

    int sensorIndex;
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::ISixAxisForceTorqueSensors * sensor;

    KDL::Rotation R_N_sensor;
    KDL::Vector toolCoM_N;
    KDL::Wrench toolWeight_0;
    KDL::Wrench initialOffset;

    bool dryRun;
    bool usingTool;
    double linGain;
    double rotGain;
    double forceDeadband;
    double torqueDeadband;
};

} // namespace roboticslab

#endif // __FT_COMPENSATION_HPP__

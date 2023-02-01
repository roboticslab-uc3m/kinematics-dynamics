// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FT_COMPENSATION_HPP__
#define __FT_COMPENSATION_HPP__

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
 * @brief Produces motion in the direction of an externally applied force
 * measured by a force-torque sensor (pretty much mimicking a classical
 * gravity compensation app).
 */
class FtCompensation : public yarp::os::RFModule,
                       public yarp::os::PeriodicThread
{
public:
    FtCompensation()
        : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes, yarp::os::PeriodicThreadClock::Absolute)
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
    bool applyImpedance(KDL::Wrench & wrench);

    yarp::dev::PolyDriver cartesianDevice;
    ICartesianControl * iCartesianControl;

    int sensorIndex;
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::ISixAxisForceTorqueSensors * sensor;

    KDL::Rotation R_N_sensor;
    KDL::Vector toolCoM_N;
    KDL::Wrench toolWeight_0;
    KDL::Wrench initialOffset;
    KDL::Frame initialPose;
    KDL::Frame previousPose;

    using cartesian_cmd = void (ICartesianControl::*)(const std::vector<double> &);
    cartesian_cmd command;

    bool dryRun;
    bool usingTool;
    bool enableImpedance;

    double linGain;
    double rotGain;

    double linStiffness;
    double rotStiffness;

    double linDamping;
    double rotDamping;

    double forceDeadband;
    double torqueDeadband;
};

} // namespace roboticslab

#endif // __FT_COMPENSATION_HPP__

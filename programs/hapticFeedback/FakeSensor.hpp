#ifndef __FAKE_SENSOR_HPP__
#define __FAKE_SENSOR_HPP__

#include <yarp/conf/version.h>

#include <yarp/os/RFModule.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include "ICartesianControl.h"
#include "FakeSensor_ParamsParser.h"

namespace roboticslab
{

/**
 * @ingroup hapticFeedback
 * @brief Simulates an FT sensor for testing purposes.
 */
class FakeSensor : public yarp::dev::DeviceDriver,
                   public yarp::dev::ISixAxisForceTorqueSensors,
                   public FakeSensor_ParamsParser
{
public:
    //  --------- DeviceDriver ---------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ISixAxisForceTorqueSensors Declarations. Implementation in ISixAxisForceTorqueSensorsImpl.cpp ---------
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    yarp::dev::ReturnValue getNrOfSixAxisForceTorqueSensors(std::size_t & num) const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    yarp::dev::ReturnValue getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    yarp::dev::ReturnValue getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & frameName) const override;
    yarp::dev::ReturnValue getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#else
    std::size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & frameName) const override;
    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;
#endif

private:
    double getDistance(const KDL::Vector & p) const;
    double getForce(double distance) const;

    yarp::dev::PolyDriver cartesianClientDevice;
    ICartesianControl * iCartesianControl {nullptr};

    KDL::Vector planeN;
};

} // namespace roboticslab

#endif // __FAKE_SENSOR_HPP__

#include "FakeSensor.hpp"

#include <cmath> // std::abs, std::min, std::sqrt

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>

#include "KdlVectorConverter.hpp"
#include "LogComponent.hpp"

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
#define CHECK_SENSOR(idx, out) do { std::size_t n; auto ret = getNrOfSixAxisForceTorqueSensors(n); if (!ret || (idx) < 0 || (idx) > n - 1) return out; } while (0)
#else
#define CHECK_SENSOR(idx, ret) do { int n = getNrOfSixAxisForceTorqueSensors(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)
#endif

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool FakeSensor::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(FS) << "Invalid parameters";
        return false;
    }

    planeN = KDL::Vector(m_planeA, m_planeB, m_planeC);

    if (planeN.Normalize() == 0.0)
    {
        yCError(FS) << "Plane ABC norm cannot be zero";
        return false;
    }

    if (m_threshold <= 0.0)
    {
        yCError(FS) << "Threshold must be positive:" << m_threshold;
        return false;
    }

    if (m_factor <= 0.0)
    {
        yCError(FS) << "Factor must be positive:" << m_factor;
        return false;
    }

    if (m_saturation <= 0.0)
    {
        yCError(FS) << "Saturation must be positive:" << m_saturation;
        return false;
    }

    if (m_local.empty())
    {
        yCError(FS) << "Local port prefix cannot be empty";
        return false;
    }

    if (m_remote.empty())
    {
        yCError(FS) << "Remote port prefix cannot be empty";
        return false;
    }

    if (m_device.empty())
    {
        yCError(FS) << "Device name cannot be empty";
        return false;
    }

    yarp::os::Property options {
        {"device", yarp::os::Value(m_device)},
        {"local", yarp::os::Value(m_local + "/sensor")},
        {"remote", yarp::os::Value(m_remote)}
    };

    if (!cartesianClientDevice.open(options))
    {
        yCError(FS) << "Failed to open Cartesian client device";
        return false;
    }

    if (!cartesianClientDevice.view(iCartesianControl))
    {
        yCError(FS) << "Failed to view Cartesian control interface";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool FakeSensor::close()
{
    return true;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue FakeSensor::getNrOfSixAxisForceTorqueSensors(std::size_t & num) const
{
    num = 1;
    return yarp::dev::ReturnValue_ok;
}
#else
std::size_t FakeSensor::getNrOfSixAxisForceTorqueSensors() const
{
    return 1;
}
#endif

// -----------------------------------------------------------------------------

yarp::dev::MAS_status FakeSensor::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return yarp::dev::MAS_status::MAS_OK;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue FakeSensor::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
#else
bool FakeSensor::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif
    name = "fake";
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue FakeSensor::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
#else
bool FakeSensor::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
#endif
{
    return getSixAxisForceTorqueSensorName(sens_index, name);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue FakeSensor::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#else
bool FakeSensor::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif

    ICartesianControl::ControllerState state;

    if (!iCartesianControl->getState(state))
    {
        yCError(FS) << "Failed to get Cartesian control state";
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    auto H_0_N = KdlVectorConverter::vectorToFrame(state.x);
    auto distance = getDistance(H_0_N.p);

    out = yarp::sig::Vector(6, 0.0);

    if (distance > m_threshold)
    {
        auto force = getForce(distance);
        auto p_N = H_0_N.M.Inverse() * (planeN * force); // planeN is referred to frame 0 (base)

        out[0] = p_N[0];
        out[1] = p_N[1];
        out[2] = p_N[2];
    }

    timestamp = yarp::os::SystemClock::nowSystem();

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

inline double FakeSensor::getDistance(const KDL::Vector & p) const
{
    return std::abs(KDL::dot(planeN, p) + m_planeD) / planeN.Norm();
}

// -----------------------------------------------------------------------------

inline double FakeSensor::getForce(double distance) const
{
    // f(x) = (factor / x) - coeff
    // f(threshold) = 0 -> coeff = factor / threshold
    return distance > 0
        ? std::min((m_factor / distance) - (m_factor / m_threshold), m_saturation)
        : m_saturation;
}

// -----------------------------------------------------------------------------

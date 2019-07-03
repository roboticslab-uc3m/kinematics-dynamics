#include "CentroidTransform.hpp"

#include <cmath>

#include <yarp/os/Time.h>

#include <kdl/frames.hpp>

#include <KdlVectorConverter.hpp>
#include <ColorDebug.h>

using namespace roboticslab;

CentroidTransform::CentroidTransform()
    : streamingDevice(NULL),
      permanenceTime(0.0)
{}

bool CentroidTransform::setTcpToCameraRotation(yarp::os::Bottle * b)
{
    if (b->size() != 3)
    {
        CD_WARNING("Bottle size must equal 3, was: %d.\n", b->size());
        return false;
    }

    double roll = b->get(0).asFloat64() * M_PI / 180.0;
    double pitch = b->get(1).asFloat64() * M_PI / 180.0;
    double yaw = b->get(2).asFloat64() * M_PI / 180.0;

    CD_INFO("centroidFrameRPY [rad]: %f %f %f\n", roll, pitch, yaw);

    rot_tcp_camera = KDL::Rotation::RPY(roll, pitch, yaw);

    return true;
}

bool CentroidTransform::acceptBottle(yarp::os::Bottle * b)
{
    if (b)
    {
        if (b->size() != 2)
        {
            CD_WARNING("Malformed input bottle, size %d (expected 2).\n", b->size());
            return false;
        }

        lastBottle = *b;
        lastAcquisition.update();
        return true;
    }

    return yarp::os::Time::now() - lastAcquisition.getTime() <= permanenceTime;
}

bool CentroidTransform::processStoredBottle() const
{
    // object centroids scaled to fit into [-1, 1] range
    double cx = lastBottle.get(0).asFloat64(); // points right
    double cy = lastBottle.get(1).asFloat64(); // points down

    std::vector<double> x;

    if (!streamingDevice->iCartesianControl->stat(x))
    {
        CD_WARNING("stat failed.\n");
        return false;
    }

    KDL::Frame H_base_tcp = KdlVectorConverter::vectorToFrame(x);

    // express camera's z axis (points "forward") in base frame
    KDL::Vector v_base = H_base_tcp.M * rot_tcp_camera * KDL::Vector(0, 0, 1);
    KDL::Frame H_base_target = KdlVectorConverter::vectorToFrame(streamingDevice->data);

    double norm = KDL::dot(H_base_target.p, v_base);

    if (norm <= 0.0)
    {
        // no action if we move away from the target (negative TCP's z axis)
        return false;
    }

    // project target vector into TCP's z axis, refer result to base frame
    H_base_target.p = v_base * norm;

    // find axis along which to rotate (in TCP frame) given pixel coords
    KDL::Vector coords(cx, cy, 0);
    KDL::Vector tcp_axis = KDL::Rotation::RotZ(KDL::PI / 2) * coords;
    KDL::Vector base_axis = H_base_tcp.M * rot_tcp_camera * tcp_axis;

    // rotate towards the target in base frame
    H_base_target.M = KDL::Rotation::Rot(base_axis, coords.Norm() * ROT_FACTOR);

    // apply changes to input transform
    streamingDevice->data = KdlVectorConverter::frameToVector(H_base_target);

    return true;
}

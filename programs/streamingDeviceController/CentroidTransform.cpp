#include "CentroidTransform.hpp"

#include <kdl/frames.hpp>

#include <KdlVectorConverter.hpp>
#include <ColorDebug.h>

using namespace roboticslab;

CentroidTransform::CentroidTransform()
    : streamingDevice(NULL)
{}

bool CentroidTransform::processBottle(const yarp::os::Bottle & b)
{
    if (b.size() != 2)
    {
        CD_WARNING("Malformed input bottle, size %d (expected 2).\n", b.size());
        return false;
    }

    // object centroids scaled to fit into [-1, 1] range
    double cx = b.get(0).asFloat64(); // points right (same as AMOR TCP's x axis)
    double cy = b.get(1).asFloat64(); // points down (same as AMOR TCP's y axis)

    std::vector<double> x;

    if (!streamingDevice->iCartesianControl->stat(x))
    {
        CD_WARNING("stat failed.\n");
        return false;
    }

    KDL::Frame H_base_tcp = KdlVectorConverter::vectorToFrame(x);

    // express TCP's z axis in base frame
    KDL::Vector v_base = H_base_tcp.M.Inverse() * KDL::Vector(0, 0, 1);
    v_base.Normalize();

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
    KDL::Rotation rot_tcp_target = KDL::Rotation::Rot(tcp_axis, coords.Norm());

    // rotate towards the target in base frame
    H_base_target.M = H_base_tcp.M * rot_tcp_target;

    streamingDevice->data = KdlVectorConverter::frameToVector(H_base_target);

    return true;
}

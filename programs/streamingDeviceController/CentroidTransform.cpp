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
    if (!b.size() != 2)
    {
        CD_WARNING("Malformed input bottle, size %d (expected 2).\n", b.size());
        return false;
    }

    // object centroids scaled to fit into [-1, 1] range
    double cx = b.get(0).asFloat64(); // points right
    double cy = b.get(1).asFloat64(); // points down

    std::vector<double> x;

    if (!streamingDevice->iCartesianControl->stat(x))
    {
        CD_WARNING("stat failed.\n");
        return false;
    }

    KDL::Frame H_base_tcp = KdlVectorConverter::vectorToFrame(x);

    // express Z axis (TCP frame) in base frame
    KDL::Vector v_base = H_base_tcp.M.Inverse() * KDL::Vector(0, 0, 1);
    v_base.Normalize();

    KDL::Frame H_d = KdlVectorConverter::vectorToFrame(streamingDevice->data);

    double norm = KDL::dot(H_d.p, v_base);

    if (norm <= 0.0)
    {
        return true;
    }

    // project desired vector into TCP's Z axis, refer result to base frame
    H_d.p = v_base * norm;

    streamingDevice->data = KdlVectorConverter::frameToVector(H_d);

    return true;
}

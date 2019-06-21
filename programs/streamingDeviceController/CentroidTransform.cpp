#include "CentroidTransform.hpp"

using namespace roboticslab;

CentroidTransform::CentroidTransform()
    : streamingDevice(NULL)
{}

bool  CentroidTransform::processBottle(const yarp::os::Bottle & b)
{
    return true;
}

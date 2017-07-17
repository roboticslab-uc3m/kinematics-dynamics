#include "PremultPorts.hpp"

#include <vector>

#include <kdl/frames.hpp>

#include <ColorDebug.hpp>

namespace roboticslab
{

/************************************************************************/

PremultPorts::PremultPorts() : KdlVectorConverter("axisAngle")
{
    return;
}

/************************************************************************/

void PremultPorts::setOutPort(yarp::os::Port* outPort)
{
    this->outPort = outPort;
}

/************************************************************************/

void PremultPorts::setIEncoders(yarp::dev::IEncoders* iEncoders)
{
    this->iEncoders = iEncoders;
    if( ! iEncoders->getAxes(&numRobotJoints) )
    {
        CD_ERROR("Could not get axes.\n");
        exit(1);
    }
    CD_SUCCESS("numRobotJoints: %d.\n",numRobotJoints);
}

/************************************************************************/

void PremultPorts::setICartesianSolver(roboticslab::ICartesianSolver* iCartesianSolver)
{
    this->iCartesianSolver = iCartesianSolver;
}

/************************************************************************/

void PremultPorts::onRead(yarp::os::Bottle& b)
{
    //printf("[PremultPorts] Got %s\n", b.toString().c_str());
    if(b.size() != 3)
    {
        CD_ERROR("For now only parsing 3-double lists\n");
        exit(1);  // case: other --> still not implemented
    }

    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        exit(1);
    }

    std::vector<double> currentX;
    iCartesianSolver->fwdKin(currentQ,currentX);

    KDL::Frame H_root_camera;
    vectorToFrame(currentX,H_root_camera);

    KDL::Frame H_camera;
    H_camera.p.data[0] = b.get(0).asDouble();
    H_camera.p.data[1] = b.get(1).asDouble();
    H_camera.p.data[2] = b.get(2).asDouble();

    KDL::Frame H_root = H_root_camera * H_camera;

    yarp::os::Bottle outB;
    outB.addDouble(H_root.p.x());
    outB.addDouble(H_root.p.y());
    outB.addDouble(H_root.p.z());
    outPort->write(outB);
}

/************************************************************************/

}  // namespace roboticslab

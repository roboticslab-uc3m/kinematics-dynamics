#include "PremultPorts.hpp"

#include <vector>

#include <kdl/frames.hpp>

#include <ColorDebug.hpp>

#include "KdlVectorConverter.hpp"

namespace roboticslab
{

/************************************************************************/

void PremultPorts::setOutPort(yarp::os::Port* outPort)
{
    this->outPort = outPort;
}

/************************************************************************/

void PremultPorts::setIEncoders(yarp::dev::IEncoders* iEncoders)
{
    this->iEncoders = iEncoders;
}

/************************************************************************/

void PremultPorts::setICartesianSolver(roboticslab::ICartesianSolver* iCartesianSolver)
{
    this->iCartesianSolver = iCartesianSolver;
}

/************************************************************************/

void PremultPorts::setNumRobotJoints(int numRobotJoints)
{
    this->numRobotJoints = numRobotJoints;
}

/************************************************************************/

void PremultPorts::setRootFrame(KDL::Frame* H_base_root)
{
    this->H_base_root = H_base_root;
}


/************************************************************************/

void PremultPorts::onRead(yarp::os::Bottle& b)
{
    CD_DEBUG("Got %s\n", b.toString().c_str());
    if(b.size() != 6)
    {
        CD_ERROR("Size error, 6-double list expected\n");
        return;
    }

    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    std::vector<double> currentX;
    iCartesianSolver->fwdKin(currentQ,currentX);

    KDL::Frame H_root_camera = KdlVectorConverter::vectorToFrame(currentX);

    KDL::Frame H_camera;
    H_camera.p.x(b.get(0).asDouble());
    H_camera.p.y(b.get(1).asDouble());
    H_camera.p.z(b.get(2).asDouble());

    KDL::Vector rotvec(b.get(3).asDouble(), b.get(4).asDouble(), b.get(5).asDouble());
    H_camera.M = KDL::Rotation::Rot(rotvec, rotvec.Norm());

    KDL::Frame H_root = (*H_base_root) * H_root_camera * H_camera;

    yarp::os::Bottle outB;
    outB.addDouble(H_root.p.x());
    outB.addDouble(H_root.p.y());
    outB.addDouble(H_root.p.z());

    KDL::Vector rotvec_root = H_root.M.GetRot();
    outB.addDouble(rotvec_root.x());
    outB.addDouble(rotvec_root.y());
    outB.addDouble(rotvec_root.z());

    outPort->write(outB);
}

/************************************************************************/

}  // namespace roboticslab

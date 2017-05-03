#include "PremultPorts.hpp"

namespace teo
{

/************************************************************************/

void PremultPorts::setOutPort(yarp::os::Port* outPort) {
    this->outPort = outPort;
}

/************************************************************************/

void PremultPorts::onRead(yarp::os::Bottle& b) {
    //printf("[PremultPorts] Got %s\n", b.toString().c_str());
    if(b.size() != 3) {
        fprintf(stderr,"[error] for now only parsing 3-double lists\n");
        exit(1);  // case: other --> still not implemented
    }

    KDL::ChainFkSolverPos_recursive oneChainFKsolver = KDL::ChainFkSolverPos_recursive(oneChain);

    //-- Should get these from encoders
    KDL::JntArray oneChainJoints = KDL::JntArray(2);
    oneChainJoints(0) = 0.0;
    oneChainJoints(1) = 0.0;

    KDL::Frame H_root_camera;
    oneChainFKsolver.JntToCart(oneChainJoints,H_root_camera);

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

}  // namespace teo

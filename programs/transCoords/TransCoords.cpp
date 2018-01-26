#include "TransCoords.hpp"

#include <string>
#include <vector>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <ColorDebug.hpp>

#include "KdlVectorConverter.hpp"

namespace roboticslab
{

/************************************************************************/
bool TransCoords::updateModule()
{
    CD_INFO("TransCoords alive...\n");
    return true;
}

/************************************************************************/
double TransCoords::getPeriod()
{
    return 2.0;  // [s]
}

/************************************************************************/

bool TransCoords::configure(yarp::os::ResourceFinder &rf)
{
    CD_INFO("TransCoords config: %s.\n", rf.toString().c_str());

    if (rf.check("help"))
    {
        CD_INFO_NO_HEADER("TransCoords options:\n");
        CD_INFO_NO_HEADER("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        return false;
    }

    if (!getMatrixFromProperties(rf.findGroup("H0").tail(), H0))
    {
        CD_ERROR("Could not parse H0.\n");
        return false;
    }

    useRobot = !rf.check("noRobot");

    if (useRobot)
    {
        std::string solverStr = rf.check("solver", yarp::os::Value(DEFAULT_SOLVER), "cartesian solver").asString();
        std::string robotStr = rf.check("robot", yarp::os::Value(DEFAULT_ROBOT), "robot device").asString();

        yarp::os::Property solverOptions;
        solverOptions.fromString(rf.toString());
        solverOptions.put("device", solverStr);

        if (!solverDevice.open(solverOptions))
        {
            CD_ERROR("solver device not valid: %s.\n", solverStr.c_str());
            return false;
        }

        if (!solverDevice.view(iCartesianSolver))
        {
            CD_ERROR("Could not view iCartesianSolver in: %s.\n", solverStr.c_str());
            return false;
        }

        yarp::os::Property robotOptions;
        robotOptions.fromString(rf.toString());
        robotOptions.put("device", robotStr);

        if (!robotDevice.open(robotOptions))
        {
            CD_ERROR("robot device not valid.\n");
            return false;
        }

        if (!robotDevice.view(iEncoders))
        {
            CD_ERROR("Could not view iEncoders.\n");
            return false;
        }

        if (!iEncoders->getAxes(&numRobotJoints))
        {
            CD_ERROR("Could not get axes.\n");
            return false;
        }

        CD_SUCCESS("numRobotJoints: %d.\n", numRobotJoints);
    }

    inPort.open("/coords:i");
    outPort.open("/coords:o");

    inPort.useCallback(*this);

    return true;
}

/************************************************************************/

bool TransCoords::interruptModule()
{
    inPort.disableCallback();
    inPort.close();

    outPort.close();

    if (useRobot)
    {
        solverDevice.close();
        robotDevice.close();
    }

    return true;
}

bool TransCoords::getMatrixFromProperties(const yarp::os::Bottle &b, KDL::Frame &frame)
{
    if (b.isNull())
    {
        return true;
    }

    if (b.size() != 16)
    {
        CD_ERROR("Unsupported matrix size (not 4x4).\n");
        return false;
    }

    if (b.get(12).asDouble() != 0 || b.get(13).asDouble() != 0 || b.get(14).asDouble() != 0 || b.get(15).asDouble() != 1)
    {
        CD_ERROR("Unsupported non-null frame components (perspective and scaling).\n");
        return false;
    }

    frame.M.UnitX(KDL::Vector(b.get(0).asDouble(), b.get(4).asDouble(), b.get(8).asDouble()));
    frame.M.UnitY(KDL::Vector(b.get(1).asDouble(), b.get(5).asDouble(), b.get(9).asDouble()));
    frame.M.UnitZ(KDL::Vector(b.get(2).asDouble(), b.get(6).asDouble(), b.get(10).asDouble()));

    frame.p.x(b.get(3).asDouble());
    frame.p.y(b.get(7).asDouble());
    frame.p.z(b.get(11).asDouble());

    return true;
}

/************************************************************************/

void TransCoords::onRead(yarp::os::Bottle &b)
{
    CD_DEBUG("Got %s\n", b.toString().c_str());

    if (b.size() != 6)
    {
        CD_ERROR("Size error, 6-double list expected\n");
        return;
    }

    std::vector<double> currentQ(numRobotJoints);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        CD_ERROR("getEncoders failed.\n");
        return;
    }

    std::vector<double> currentX;

    if (!iCartesianSolver->fwdKin(currentQ, currentX))
    {
        CD_ERROR("fdwkin error.\n");
        return;
    }

    KDL::Frame H_0_N = KdlVectorConverter::vectorToFrame(currentX);

    KDL::Frame HN;
    HN.p.x(b.get(0).asDouble());
    HN.p.y(b.get(1).asDouble());
    HN.p.z(b.get(2).asDouble());

    KDL::Vector rotvec(b.get(3).asDouble(), b.get(4).asDouble(), b.get(5).asDouble());
    HN.M = KDL::Rotation::Rot(rotvec, rotvec.Norm());

    KDL::Frame H = H0 * H_0_N * HN;

    yarp::os::Bottle &outB = outPort.prepare();
    outB.addDouble(H.p.x());
    outB.addDouble(H.p.y());
    outB.addDouble(H.p.z());

    KDL::Vector rotvec_root = H.M.GetRot();
    outB.addDouble(rotvec_root.x());
    outB.addDouble(rotvec_root.y());
    outB.addDouble(rotvec_root.z());

    outPort.write();
}

/************************************************************************/

}  // namespace roboticslab

#include "TransCoords.hpp"

#include <string>
#include <vector>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>

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
        CD_INFO_NO_HEADER("\t--fixedH (16 doubles) <- skip robot, use fixed H matrix instead\n");
        CD_INFO_NO_HEADER("\t--solver \"name\" <- solver device [KdlSolver]\n");
        CD_INFO_NO_HEADER("\t--robot \"name\" <- robot device [remote_controlboard]\n");
        CD_INFO_NO_HEADER("\t--angleRepr \"name\" <- angle representation [axisAngleScaled]\n");
        return false;
    }

    useRobot = !rf.check("fixedH");

    if (useRobot)
    {
        CD_INFO("Using robot parameters.\n");

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

        yarp::dev::IControlLimits * iControlLimits;

        if (!robotDevice.view(iControlLimits))
        {
            CD_ERROR("Could not view iControlLimits.\n");
            return false;
        }

        std::vector<double> qMin, qMax;

        for (int i = 0; i < numRobotJoints; i++)
        {
            double min, max;
            iControlLimits->getLimits(i, &min, &max);
            qMin.push_back(min);
            qMax.push_back(max);
        }

        if (qMin[0] != qMax[0])
        {
            iCartesianSolver->setLimits(qMin, qMax);
        }

        CD_SUCCESS("numRobotJoints: %d.\n", numRobotJoints);
    }
    else
    {
        CD_INFO("Using fixedH parameters.\n");

        if (!getMatrixFromProperties(rf.findGroup("fixedH").tail(), fixedH))
        {
            CD_ERROR("Could not parse fixedH.\n");
            return false;
        }
    }

    std::string angleReprStr = rf.check("angleRepr", yarp::os::Value(DEFAULT_ANGLE_REPR), "angle representation").asString();

    if (!KinRepresentation::parseEnumerator(angleReprStr, &orient))
    {
        CD_WARNING("Unknown angleRepr \"%s\", falling back to default.\n", angleReprStr.c_str());
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
    if (b.isNull() || b.size() == 0)
    {
        return true;
    }

    if (!b.get(0).isList())
    {
        CD_ERROR("Unsupported bottle format, data must be a list.\n");
        return false;
    }

    yarp::os::Bottle *l = b.get(0).asList();

    if (l->size() != 16)
    {
        CD_ERROR("Unsupported matrix size (not 4x4).\n");
        return false;
    }

    if (l->get(12).asDouble() != 0 || l->get(13).asDouble() != 0 || l->get(14).asDouble() != 0 || l->get(15).asDouble() != 1)
    {
        CD_ERROR("Unsupported non-null frame components (perspective and scaling).\n");
        return false;
    }

    frame.M.UnitX(KDL::Vector(l->get(0).asDouble(), l->get(4).asDouble(), l->get(8).asDouble()));
    frame.M.UnitY(KDL::Vector(l->get(1).asDouble(), l->get(5).asDouble(), l->get(9).asDouble()));
    frame.M.UnitZ(KDL::Vector(l->get(2).asDouble(), l->get(6).asDouble(), l->get(10).asDouble()));

    frame.p.x(l->get(3).asDouble());
    frame.p.y(l->get(7).asDouble());
    frame.p.z(l->get(11).asDouble());

    return true;
}

/************************************************************************/

void TransCoords::onRead(yarp::os::Bottle &b)
{
    CD_DEBUG("Got %s\n", b.toString().c_str());

    std::vector<double> x;

    for (int i = 0; i < b.size(); i++)
    {
        x.push_back(b.get(i).asDouble());
    }

    if (!KinRepresentation::encodePose(x, x, KinRepresentation::CARTESIAN, orient) || x.size() != 6)
    {
        CD_ERROR("encodePose failed.\n");
        return;
    }

    KDL::Frame HN;
    HN.p.x(x[0]);
    HN.p.y(x[1]);
    HN.p.z(x[2]);

    KDL::Vector rotvec(x[3], x[4], x[5]);
    HN.M = KDL::Rotation::Rot(rotvec, rotvec.Norm());

    KDL::Frame H;

    if (useRobot)
    {
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

        H = H_0_N * HN;
    }
    else
    {
        H = fixedH * HN;
    }

    yarp::os::Bottle &outB = outPort.prepare();
    outB.clear();

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

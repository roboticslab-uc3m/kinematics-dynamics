#include "TransCoords.hpp"

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <yarp/dev/IEncoders.h>

#include <ColorDebug.hpp>

#include "ICartesianSolver.h"

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

        roboticslab::ICartesianSolver* iCartesianSolver;

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

        yarp::dev::IEncoders* iEncoders;

        if (!robotDevice.view(iEncoders))
        {
            CD_ERROR("Could not view iEncoders.\n");
            return false;
        }

        int numRobotJoints;

        if (!iEncoders->getAxes(&numRobotJoints))
        {
            CD_ERROR("Could not get axes.\n");
            return false;
        }

        CD_SUCCESS("numRobotJoints: %d.\n", numRobotJoints);

        premultPorts.setIEncoders(iEncoders);
        premultPorts.setICartesianSolver(iCartesianSolver);
        premultPorts.setNumRobotJoints(numRobotJoints);
    }

    outPort.open("/coords:o");
    premultPorts.setOutPort(&outPort);
    premultPorts.setRootFrame(&H0);
    premultPorts.open("/coords:i");
    premultPorts.useCallback();

    return true;
}

/************************************************************************/

bool TransCoords::interruptModule()
{
    premultPorts.disableCallback();
    premultPorts.close();
    outPort.close();

    if (useRobot)
    {
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

}  // namespace roboticslab

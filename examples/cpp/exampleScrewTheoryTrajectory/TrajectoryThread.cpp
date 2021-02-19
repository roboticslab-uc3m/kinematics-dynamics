// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryThread.hpp"

#include <algorithm>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <KdlVectorConverter.hpp>
#include <KinematicRepresentation.hpp>

namespace
{
    void printCartesianCoordinates(const KDL::Frame & H, double movementTime)
    {
        KDL::Vector p = H.p;
        double roll, pitch, yaw;
        H.M.GetRPY(roll, pitch, yaw);

        yInfo("GOAL -> p: %f %f %f, r: %f %f %f [%f]", p.x(), p.y(), p.z(), roll, pitch, yaw, movementTime);
    }
}

bool TrajectoryThread::threadInit()
{
    startTime = yarp::os::Time::now();
    return iEncoders->getAxes(&axes);
}

void TrajectoryThread::run()
{
    double movementTime = yarp::os::Time::now() - startTime;

    std::vector<double> position;
    iCartTrajectory->getPosition(movementTime, position);

    KDL::Frame H_S_T = roboticslab::KdlVectorConverter::vectorToFrame(position);
    printCartesianCoordinates(H_S_T, movementTime);

    std::vector<KDL::JntArray> solutions;

    if (!ikProblem->solve(H_S_T, solutions))
    {
        yWarning() << "IK exact solution not found";
    }

    if (!ikConfig->configure(solutions))
    {
        yError() << "IK solutions out of joint limits";
        return;
    }

    KDL::JntArray q(axes);

    if (!iEncoders->getEncoders(q.data.data()))
    {
        yError() << "getEncoders() failed";
        return;
    }

    for (int i = 0; i < q.rows(); i++)
    {
        q(i) = roboticslab::KinRepresentation::degToRad(q(i));
    }

    if (!ikConfig->findOptimalConfiguration(q))
    {
        yError() << "Optimal configuration not found";
        return;
    }

    KDL::JntArray solution;

    ikConfig->retrievePose(solution);

    std::vector<double> refs(solution.data.data(), solution.data.data() + solution.data.size());
    std::transform(refs.begin(), refs.end(), refs.begin(), roboticslab::KinRepresentation::radToDeg);
    yInfo() << "IK ->" << refs;

    iPosDirect->setPositions(refs.data());
}

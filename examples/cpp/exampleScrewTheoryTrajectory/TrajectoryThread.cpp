// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryThread.hpp"

#include <algorithm>
#include <vector>

#include <yarp/os/Time.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <KdlVectorConverter.hpp>
#include <KinematicRepresentation.hpp>

#include <ColorDebug.h>

namespace
{
    void printCartesianCoordinates(const KDL::Frame & H, double movementTime)
    {
        KDL::Vector p = H.p;
        double roll, pitch, yaw;
        H.M.GetRPY(roll, pitch, yaw);

        CD_DEBUG_NO_HEADER("GOAL -> p: %f %f %f, r: %f %f %f [%f]\n",
                p.x(), p.y(), p.z(), roll, pitch, yaw, movementTime);
    }

    void printJointCoordinates(const std::vector<double> & q)
    {
        CD_INFO_NO_HEADER("IK ->");

        for (int i = 0; i < q.size(); i++)
        {
            CD_INFO_NO_HEADER(" %f", q[i]);
        }

        CD_INFO_NO_HEADER("\n");
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
        CD_WARNING("IK exact solution not found.\n");
    }

    if (!ikConfig->configure(solutions))
    {
        CD_ERROR("IK solutions out of joint limits.\n");
        return;
    }

    KDL::JntArray q(axes);

    if (!iEncoders->getEncoders(q.data.data()))
    {
        CD_ERROR("getEncoders() failed.\n");
        return;
    }

    for (int i = 0; i < q.rows(); i++)
    {
        q(i) = roboticslab::KinRepresentation::degToRad(q(i));
    }

    if (!ikConfig->findOptimalConfiguration(q))
    {
        CD_ERROR("Optimal configuration not found.\n");
        return;
    }

    KDL::JntArray solution;

    ikConfig->retrievePose(solution);

    std::vector<double> refs(solution.data.data(), solution.data.data() + solution.data.size());
    std::transform(refs.begin(), refs.end(), refs.begin(), roboticslab::KinRepresentation::radToDeg);
    printJointCoordinates(refs);

    iPosDirect->setPositions(refs.data());
}

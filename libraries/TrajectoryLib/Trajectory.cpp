// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Trajectory.hpp"

// -----------------------------------------------------------------------------

teo::Trajectory::Trajectory()
{
    angleRepr = "axisAngle";
}

// -----------------------------------------------------------------------------

bool teo::Trajectory::getX(const double movementTime, std::vector<double>& desiredX)
{
    //KDL::Frame desiredX = trajectory->Pos(movementTime);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::Trajectory::getXdot(const double movementTime, std::vector<double>& desiredXdot)
{
    //KDL::Twist desiredXdot = trajectory->Vel(movementTime);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::Trajectory::newLine(const std::vector<double> &src, const std::vector<double> &dest)
{
    KDL::Frame srcFrame, destFrame;
    vectorToFrame(src,srcFrame);
    vectorToFrame(dest,destFrame);
    _orient = new KDL::RotationalInterpolation_SingleAxis();
    double _eqradius = 1; //0.000001;
    bool _aggregate = true;
    double duration = DEFAULT_DURATION;

    KDL::Path_Line testPathLine(srcFrame, destFrame, _orient, _eqradius, _aggregate);
    KDL::VelocityProfile_Trap testVelocityProfile(DEFAULT_MAXVEL, DEFAULT_MAXACC);
//    KDL::Trajectory_Segment testTrajectory(testPathLine.Clone(), testVelocityProfile.Clone(), duration, _aggregate);
    currentTrajectory = new KDL::Trajectory_Segment(testPathLine.Clone(), testVelocityProfile.Clone(), duration, _aggregate);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::Trajectory::deleteLine()
{
    delete _orient;
    _orient = 0;
    delete currentTrajectory;
    currentTrajectory = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::Trajectory::vectorToFrame(const std::vector<double> &x, KDL::Frame& f) {

    f.p.data[0]=x[0];
    f.p.data[1]=x[1];
    f.p.data[2]=x[2];

    if (angleRepr == "axisAngle") {
        f.M = KDL::Rotation::Rot(KDL::Vector(x[3],x[4],x[5]),toRad(x[6]));
    }
    else if (angleRepr == "eulerYZ")  //-- like ASIBOT
    {
        f.M = KDL::Rotation::EulerZYZ(::atan2(x[1],x[0]),toRad(x[3]), toRad(x[4]));
    }
    else if (angleRepr == "eulerZYZ")
    {
        f.M = KDL::Rotation::EulerZYZ(toRad(x[3]), toRad(x[4]), toRad(x[5]));
    }
    else if (angleRepr == "RPY")
    {
        f.M = KDL::Rotation::RPY(toRad(x[3]), toRad(x[4]), toRad(x[5]));
    }
    else  //-- No known angle repr.
    {
        CD_WARNING("angleRepr unknown %s\n",angleRepr.c_str());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::Trajectory::frameToVector(const KDL::Frame& f, std::vector<double> &x) {

    //-- Fill angle first, then 0-2 for position.

    if (angleRepr == "axisAngle")
    {
        KDL::Vector rotVector = f.M.GetRot();
        x.resize(7);
        x[6] = toDeg(f.M.GetRotAngle(rotVector));  // Normalizes as colateral effect
        x[3] = rotVector[0];
        x[4] = rotVector[1];
        x[5] = rotVector[2];
    }
    else if (angleRepr == "eulerYZ") //-- like ASIBOT
    {
        double alfa, beta, gamma;
        f.M.GetEulerZYZ(alfa, beta, gamma);
        x.resize(5);
        x[3] = toDeg(beta);
        x[4] = toDeg(gamma);
    }
    else if (angleRepr == "eulerZYZ")
    {
        double alfa, beta, gamma;
        f.M.GetEulerZYZ(alfa, beta, gamma);
        x.resize(6);
        x[3] = toDeg(alfa);
        x[4] = toDeg(beta);
        x[5] = toDeg(gamma);
    }
    else if (angleRepr == "RPY")
    {
        double alfa, beta, gamma;
        f.M.GetRPY(alfa, beta, gamma);
        x.resize(6);
        x[3] = toDeg(alfa);
        x[4] = toDeg(beta);
        x[5] = toDeg(gamma);
    }
    else  //-- No known angle repr.
    {
        CD_WARNING("angleRepr unknown %s\n",angleRepr.c_str());
    }

    x[0] = f.p.data[0];
    x[1] = f.p.data[1];
    x[2] = f.p.data[2];

    return true;
}

// -----------------------------------------------------------------------------

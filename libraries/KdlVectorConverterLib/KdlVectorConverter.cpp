// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlVectorConverter.hpp"

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

KDL::Frame roboticslab::KdlVectorConverter::vectorToFrame(const std::vector<double> &x)
{
    if (x.size() != 6)
    {
        CD_WARNING("Size mismatch; expected: 6, was: %d\n", x.size());
        return KDL::Frame::Identity();
    }

    KDL::Frame f;

    f.p.x(x[0]);
    f.p.y(x[1]);
    f.p.z(x[2]);

    KDL::Vector rotvec(x[3], x[4], x[5]);

    f.M = KDL::Rotation::Rot(rotvec, rotvec.Norm());

    return f;
}

// -----------------------------------------------------------------------------

std::vector<double> roboticslab::KdlVectorConverter::frameToVector(const KDL::Frame& f)
{
    std::vector<double> x(6);

    x[0] = f.p.x();
    x[1] = f.p.y();
    x[2] = f.p.z();

    KDL::Vector rotVector = f.M.GetRot();

    x[3] = rotVector.x();
    x[4] = rotVector.y();
    x[5] = rotVector.z();

    return x;
}

// -----------------------------------------------------------------------------

KDL::Twist roboticslab::KdlVectorConverter::vectorToTwist(const std::vector<double> &xdot)
{
    if (xdot.size() != 6)
    {
        CD_WARNING("Size mismatch; expected: 6, was: %d\n", xdot.size());
        return KDL::Twist::Zero();
    }

    KDL::Twist t;

    t.vel.x(xdot[0]);
    t.vel.y(xdot[1]);
    t.vel.z(xdot[2]);

    t.rot.x(xdot[3]);
    t.rot.y(xdot[4]);
    t.rot.z(xdot[5]);

    return t;
}

// -----------------------------------------------------------------------------

std::vector<double> roboticslab::KdlVectorConverter::twistToVector(const KDL::Twist& t)
{
    std::vector<double> xdot(6);

    xdot[0] = t.vel.x();
    xdot[1] = t.vel.y();
    xdot[2] = t.vel.z();

    xdot[3] = t.rot.x();
    xdot[4] = t.rot.y();
    xdot[5] = t.rot.z();

    return xdot;
}

// -----------------------------------------------------------------------------

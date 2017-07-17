// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlVectorConverter.hpp"

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

roboticslab::KdlVectorConverter::KdlVectorConverter(std::string angleRepr)
{
    this->angleRepr = angleRepr;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlVectorConverter::vectorToFrame(const std::vector<double> &x, KDL::Frame& f) {

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

bool roboticslab::KdlVectorConverter::frameToVector(const KDL::Frame& f, std::vector<double> &x) {

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

bool roboticslab::KdlVectorConverter::twistToVector(const KDL::Twist& t, std::vector<double> &xdot)
{
    xdot.resize(6);

    for(unsigned int i=0; i<6; i++)
        xdot[i] = t[i];

    return true;
}

// -----------------------------------------------------------------------------

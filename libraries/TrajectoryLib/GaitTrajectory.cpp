
#include "GaitTrajectory.hpp"

teo::GaitTrajectory::GaitTrajectory()
{

    //revisar esos valores-->
    steps = new teo::GaitSupportPoligon(kin::Pose(0,-0.3,-1),kin::Pose(0,+0.3,-1));//<--revisar estos valores
    steps->AddStepForward(1);
    steps->GetTrajectories(trf,tlf);

}

teo::GaitTrajectory::~GaitTrajectory()
{
    delete steps;
}

bool teo::GaitTrajectory::getX(const double movementTime, std::vector<double>& x)
{

    double px,py,pz;
    double rx,ry,rz,ang;

    kin::Pose sample;

    trf.GetSample(movementTime,sample);
    sample.GetPosition(px,py,pz);
    sample.GetRotation(rx,ry,rz,ang);

    x.clear();

    x.push_back(px);
    x.push_back(py);
    x.push_back(pz);

    x.push_back(rx);
    x.push_back(ry);
    x.push_back(rz);
    x.push_back(ang*180/M_PI);

    tlf.GetSample(movementTime,sample);
    sample.GetPosition(px,py,pz);
    sample.GetRotation(rx,ry,rz,ang);

    x.push_back(px);
    x.push_back(py);
    x.push_back(pz);

    x.push_back(rx);
    x.push_back(ry);
    x.push_back(rz);
    x.push_back(ang*180/M_PI);


    return true;
}

// -----------------------------------------------------------------------------

bool teo::GaitTrajectory::getXdot(const double movementTime, std::vector<double>& xdot)
{

    double px,py,pz;
    double rx,ry,rz,ang;

    kin::Pose sampleVelocity;

    trf.GetSampleVelocity(movementTime,sampleVelocity);
    sampleVelocity.GetPosition(px,py,pz);
    sampleVelocity.GetRotation(rx,ry,rz,ang);

    xdot.clear();

    xdot.push_back(px);
    xdot.push_back(py);
    xdot.push_back(pz);

    xdot.push_back(rx);
    xdot.push_back(ry);
    xdot.push_back(rz);
    xdot.push_back(ang*180/M_PI);

    tlf.GetSampleVelocity(movementTime,sampleVelocity);
    sampleVelocity.GetPosition(px,py,pz);
    sampleVelocity.GetRotation(rx,ry,rz,ang);

    xdot.push_back(px);
    xdot.push_back(py);
    xdot.push_back(pz);

    xdot.push_back(rx);
    xdot.push_back(ry);
    xdot.push_back(rz);
    xdot.push_back(ang*180/M_PI);
    return true;
}

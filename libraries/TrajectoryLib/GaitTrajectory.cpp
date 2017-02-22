
#include "GaitTrajectory.hpp"

teo::GaitTrajectory::GaitTrajectory()
{

    //revisados estos valores-->

    kin::Pose rf(0,-0.1285,-0.845);
    rf.ChangeRotation(0,1,0,M_PI/2);
    //rf.ChangeRotation(1,0,0,M_PI);

    std::cout << "right foot : " << rf.GetX() << "," << rf.GetY() << "," << rf.GetZ() << "," ;
    std::cout << std::endl;

    kin::Pose lf(0,+0.1285,-0.845);
    lf.ChangeRotation(0,1,0,-M_PI/2);

    std::cout << "left foot : " << rf.GetX() << "," << rf.GetY() << "," << rf.GetZ() << "," ;
    std::cout << std::endl;

    //revisados estos valores<--

    steps = new GaitSupportPoligon(rf,lf);
    steps->SetSwingParameters(0.05,0.05); //(swing distance, swing height). revisar valores
    steps->SetHipParameters(0.20,0.07); //(hip sideshift, hip squat). revisar estos valores
    steps->BeforeStep();
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

    if (trf.GetSample(movementTime,sample)==false)
    {
        //no data for that time, no moving.
        x=lastGoodX;
        return true;
    }
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

    std::cout << "> rfX: " << px << ","<< py << ","<< pz << ",";
    std::cout << "> rot: " << rx << ","<< ry << ","<< rz << ","<< ang*180/M_PI << ",";
    std::cout << std::endl;

    if (!tlf.GetSample(movementTime,sample))
    {
        //no data for that time, no moving.
        x=lastGoodX;
        return true;
    }
    sample.GetPosition(px,py,pz);
    sample.GetRotation(rx,ry,rz,ang);

    x.push_back(px);
    x.push_back(py);
    x.push_back(pz);

    x.push_back(rx);
    x.push_back(ry);
    x.push_back(rz);
    x.push_back(ang*180/M_PI);

    std::cout << "> lfX: " << px << ","<< py << ","<< pz << ",";
    std::cout << "> rot: " << rx << ","<< ry << ","<< rz << ","<< ang*180/M_PI << ",";
    std::cout << std::endl;

    lastGoodX = x;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::GaitTrajectory::getXdot(const double movementTime, std::vector<double>& xdot)
{

    double px,py,pz;
    double rx,ry,rz,ang;

    kin::Pose sampleVelocity;

    if(!trf.GetSampleVelocity(movementTime,sampleVelocity))
    {
        //no velocities for that time.
        xdot=std::vector <double> (14,0);
        return true;
    }
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

    std::cout << "> rfXdot: " << px << ","<< py << ","<< pz << ",";
    std::cout << "> rotdot: " << rx << ","<< ry << ","<< rz << ","<< ang*180/M_PI << ",";
    std::cout << std::endl;


    if (!tlf.GetSampleVelocity(movementTime,sampleVelocity))
    {
        //no velocities for that time.
        xdot=std::vector <double> (14,0);
        return true;
    }
    sampleVelocity.GetPosition(px,py,pz);
    sampleVelocity.GetRotation(rx,ry,rz,ang);

    xdot.push_back(px);
    xdot.push_back(py);
    xdot.push_back(pz);

    xdot.push_back(rx);
    xdot.push_back(ry);
    xdot.push_back(rz);
    xdot.push_back(ang*180/M_PI);

    std::cout << "> lfXdot: " << px << ","<< py << ","<< pz << ",";
    std::cout << "> rotdot: " << rx << ","<< ry << ","<< rz << ","<< ang*180/M_PI << ",";
    std::cout << std::endl;


    return true;
}

#ifndef GAITTRAJECTORY_H
#define GAITTRAJECTORY_H

#include <vector>

#include "Trajectory.hpp"

#include "GaitLipm.h"
#include "tools.h"

namespace roboticslab
{

class GaitTrajectory : public roboticslab::Trajectory
{
public:
    GaitTrajectory();
    ~GaitTrajectory();

    virtual bool getX(const double movementTime, std::vector<double>& x);
    virtual bool getXdot(const double movementTime, std::vector<double>& xdot);

private:

    //every gait operation start with instantiation and initialization of a Gait child class
    //GaitSupportPoligon * steps;
    GaitLipm * steps;

    tra::SpaceTrajectory trf,tlf;
    std::vector<double> lastGoodX;

    //GaitSupportPoligon steps(kin::Pose(0,-0.3,-1),kin::Pose(0,+0.3,-1));


};

}//namespace roboticslab end

#endif // GAITTRAJECTORY_H

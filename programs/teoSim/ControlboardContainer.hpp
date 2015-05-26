// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CONTROLBOARD_CONTAINER_HPP__
#define __CONTROLBOARD_CONTAINER_HPP__

#include <yarp/os/all.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/sig/all.h>

#include <openrave-core.h>

#include <stdio.h>
#include <iostream>
#include <sstream>

#include "ColorDebug.hpp"

using namespace std;

using namespace yarp::os;
using namespace yarp::dev;

using namespace OpenRAVE;

namespace teo
{

class ControlboardContainer {

    public:

        bool start();
        bool stop();
        void setFatherRobotIdx(int value);
        void setManipulatorWrapperName(const std::string &value);
        void push_back(int robotJointIdx);
        void push_back_tr(double robotJointTr);

        const int getFatherRobotIdx();
        std::vector<int>& getVectorOfJointIdxRef();
        std::vector<double>& getVectorOfJointPosRef();
        std::vector<double>& getVectorOfJointTrRef();

protected:

        int fatherRobotIdx;
        std::vector<int> vectorOfJointIdx;
        std::vector<double> vectorOfJointPos;
        std::vector<double> vectorOfJointTr;
        PolyDriver dd;
        IEncoders *encs;
        std::string manipulatorWrapperName;
};

}  // namsepace teo

#endif  // __CONTROLBOARD_CONTAINER_HPP__


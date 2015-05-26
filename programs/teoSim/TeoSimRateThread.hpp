// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_SIM_RATE_THREAD_HPP__
#define __TEO_SIM_RATE_THREAD_HPP__

#include <yarp/os/all.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>

#include <openrave-core.h>

#include <stdio.h>
#include <iostream>
#include <sstream>

#include "ControlboardContainer.hpp"

#define NULL_JMC_MS 20

#define DEFAULT_EXTERN_OBJ "none"
//#define DEFAULT_EXTERN_OBJ "redCan"  // loads plugin

using namespace std;

using namespace yarp::os;
using namespace yarp::dev;

using namespace OpenRAVE;

namespace teo
{

/**
 * @ingroup teoSim
 * @brief Helper class, implements the yarp::os::RateThread.
 */
class TeoSimRateThread : public RateThread {
     public:

        // Set the Thread Rate in the class constructor
        TeoSimRateThread() : RateThread(NULL_JMC_MS) {}  // In ms

        void setEnvironmentPtr(const EnvironmentBasePtr& environmentPtr) {
            this->environmentPtr = environmentPtr;
        }

        void setPtrVectorOfManipulatorWrapperPtr(vector< ControlboardContainer* > * ptrVectorOfManipulatorWrapperPtr) {
            this->ptrVectorOfManipulatorWrapperPtr = ptrVectorOfManipulatorWrapperPtr;
        }

        void setPtrVectorOfRobotPtr(vector< RobotBasePtr > * ptrVectorOfRobotPtr) {
            this->ptrVectorOfRobotPtr = ptrVectorOfRobotPtr;
        }

        void setPtrVectorOfSensorPtrForCameras(vector< SensorBasePtr > * ptrVectorOfSensorPtrForCameras) {
            this->ptrVectorOfSensorPtrForCameras = ptrVectorOfSensorPtrForCameras;
        }

        void setPtrVectorOfSensorPtrForLasers(vector< SensorBasePtr > * ptrVectorOfSensorPtrForLasers) {
            this->ptrVectorOfSensorPtrForLasers = ptrVectorOfSensorPtrForLasers;
        }

        void setPtrVectorOfCameraSensorDataPtr(vector< boost::shared_ptr<SensorBase::CameraSensorData> > * ptrVectorOfCameraSensorDataPtr) {
            this->ptrVectorOfCameraSensorDataPtr = ptrVectorOfCameraSensorDataPtr;
        }

        void setPtrVectorOfLaserSensorDataPtr(vector< boost::shared_ptr<SensorBase::LaserSensorData> > * setPtrVectorOfLaserSensorDataPtr) {
            this->ptrVectorOfLaserSensorDataPtr = setPtrVectorOfLaserSensorDataPtr;
        }

        void setPtrVectorOfCameraWidth(vector<int> * ptrVectorOfCameraWidth) {
            this->ptrVectorOfCameraWidth = ptrVectorOfCameraWidth;
        }

        void setPtrVectorOfCameraHeight(vector<int> * ptrVectorOfCameraHeight) {
            this->ptrVectorOfCameraHeight = ptrVectorOfCameraHeight;
        }

        void setPtrVectorOfRgbPortPtr(vector< BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* > * ptrVectorOfRgbPortPtr) {
            this->ptrVectorOfRgbPortPtr = ptrVectorOfRgbPortPtr;
        }

        void setPtrVectorOfIntPortPtr(vector< BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> >* > * ptrVectorOfIntPortPtr) {
            this->ptrVectorOfIntPortPtr = ptrVectorOfIntPortPtr;
        }

    // -------- RateThread declarations. Implementation in RateThreadImpl.cpp --------

        /**
         * Initialization method. The thread executes this function
         * when it starts and before "run". This is a good place to
         * perform initialization tasks that need to be done by the
         * thread itself (device drivers initialization, memory
         * allocation etc). If the function returns false the thread
         * quits and never calls "run". The return value of threadInit()
         * is notified to the class and passed as a parameter
         * to afterStart(). Note that afterStart() is called by the
         * same thread that is executing the "start" method.
         */
        bool threadInit();

        /**
         * Loop function. This is the thread itself.
         */
        void run();

    // ------------------------------- Protected -------------------------------------
    protected:
        //
        double jmcMs;
        double lastTime;
        //
        // Rave-specific //
        EnvironmentBasePtr environmentPtr;
        vector<RobotBasePtr> * ptrVectorOfRobotPtr;
        //
        std::vector< SensorBasePtr > * ptrVectorOfSensorPtrForCameras;
        std::vector< SensorBasePtr > * ptrVectorOfSensorPtrForLasers;
        std::vector< boost::shared_ptr<SensorBase::CameraSensorData> > * ptrVectorOfCameraSensorDataPtr;
        std::vector< boost::shared_ptr<SensorBase::LaserSensorData> > * ptrVectorOfLaserSensorDataPtr;
        std::vector<int> * ptrVectorOfCameraWidth;
        std::vector<int> * ptrVectorOfCameraHeight;
        std::vector< BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* > * ptrVectorOfRgbPortPtr;
        std::vector< BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> >* > * ptrVectorOfIntPortPtr;
        //
        /** Vector to store pointers to ManipulatorWrapper objects */
        std::vector < ControlboardContainer* > * ptrVectorOfManipulatorWrapperPtr;
};

}  // namespace teo

#endif  // __TEO_SIM_HPP__


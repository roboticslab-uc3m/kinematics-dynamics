// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_COLLISION_AVOIDANCE_RATE_THREAD_HPP__
#define __TEO_COLLISION_AVOIDANCE_RATE_THREAD_HPP__

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

namespace teo
{

/**
 * @ingroup teoCollisionAvoidance
 *
 * @brief Helper class, implements the yarp::os::RateThread.
 */
class TeoCollisionAvoidanceRateThread : public yarp::os::RateThread {
     public:

        // Set the Thread Rate in the class constructor
        TeoCollisionAvoidanceRateThread() : RateThread(NULL_JMC_MS) {}  // In ms

        void setEnvironmentPtr(const OpenRAVE::EnvironmentBasePtr& environmentPtr) {
            this->environmentPtr = environmentPtr;
        }

        void setPtrVectorOfControlboardContainerPtr(std::vector< ControlboardContainer* > * ptrVectorOfManipulatorWrapperPtr) {
            this->ptrVectorOfManipulatorWrapperPtr = ptrVectorOfManipulatorWrapperPtr;
        }

        void setPtrVectorOfRobotPtr(std::vector< OpenRAVE::RobotBasePtr > * ptrVectorOfRobotPtr) {
            this->ptrVectorOfRobotPtr = ptrVectorOfRobotPtr;
        }

        void setPtrVectorOfSensorPtrForCameras(std::vector< OpenRAVE::SensorBasePtr > * ptrVectorOfSensorPtrForCameras) {
            this->ptrVectorOfSensorPtrForCameras = ptrVectorOfSensorPtrForCameras;
        }

        void setPtrVectorOfSensorPtrForLasers(std::vector< OpenRAVE::SensorBasePtr > * ptrVectorOfSensorPtrForLasers) {
            this->ptrVectorOfSensorPtrForLasers = ptrVectorOfSensorPtrForLasers;
        }

        void setPtrVectorOfSensorPtrForForce6Ds(std::vector< OpenRAVE::SensorBasePtr > * ptrVectorOfSensorPtrForForce6Ds) {
            this->ptrVectorOfSensorPtrForForce6Ds = ptrVectorOfSensorPtrForForce6Ds;
        }

        void setPtrVectorOfCameraSensorDataPtr(std::vector< boost::shared_ptr<OpenRAVE::SensorBase::CameraSensorData> > * ptrVectorOfCameraSensorDataPtr) {
            this->ptrVectorOfCameraSensorDataPtr = ptrVectorOfCameraSensorDataPtr;
        }

        void setPtrVectorOfLaserSensorDataPtr(std::vector< boost::shared_ptr<OpenRAVE::SensorBase::LaserSensorData> > * setPtrVectorOfLaserSensorDataPtr) {
            this->ptrVectorOfLaserSensorDataPtr = setPtrVectorOfLaserSensorDataPtr;
        }

        void setPtrVectorOfForce6DSensorDataPtr(std::vector< boost::shared_ptr<OpenRAVE::SensorBase::Force6DSensorData> > * setPtrVectorOfForce6DSensorDataPtr) {
            this->ptrVectorOfForce6DSensorDataPtr = setPtrVectorOfForce6DSensorDataPtr;
        }

        void setPtrVectorOfCameraWidth(std::vector<int> * ptrVectorOfCameraWidth) {
            this->ptrVectorOfCameraWidth = ptrVectorOfCameraWidth;
        }

        void setPtrVectorOfCameraHeight(std::vector<int> * ptrVectorOfCameraHeight) {
            this->ptrVectorOfCameraHeight = ptrVectorOfCameraHeight;
        }

        void setPtrVectorOfRgbPortPtr(std::vector< yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* > * ptrVectorOfRgbPortPtr) {
            this->ptrVectorOfRgbPortPtr = ptrVectorOfRgbPortPtr;
        }

        void setPtrVectorOfIntPortPtr(std::vector< yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> >* > * ptrVectorOfIntPortPtr) {
            this->ptrVectorOfIntPortPtr = ptrVectorOfIntPortPtr;
        }

        void setPtrVectorOfForce6DPortPtr(std::vector< yarp::os::BufferedPort<yarp::os::Bottle >* > * ptrVectorOfForce6DPortPtr) {
            this->ptrVectorOfForce6DPortPtr = ptrVectorOfForce6DPortPtr;
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
        OpenRAVE::EnvironmentBasePtr environmentPtr;
        std::vector<OpenRAVE::RobotBasePtr> * ptrVectorOfRobotPtr;
        //
        std::vector< OpenRAVE::SensorBasePtr > * ptrVectorOfSensorPtrForCameras;
        std::vector< OpenRAVE::SensorBasePtr > * ptrVectorOfSensorPtrForLasers;
        std::vector< OpenRAVE::SensorBasePtr > * ptrVectorOfSensorPtrForForce6Ds;
        std::vector< boost::shared_ptr<OpenRAVE::SensorBase::CameraSensorData> > * ptrVectorOfCameraSensorDataPtr;
        std::vector< boost::shared_ptr<OpenRAVE::SensorBase::LaserSensorData> > * ptrVectorOfLaserSensorDataPtr;
        std::vector< boost::shared_ptr<OpenRAVE::SensorBase::Force6DSensorData> > * ptrVectorOfForce6DSensorDataPtr;
        std::vector<int> * ptrVectorOfCameraWidth;
        std::vector<int> * ptrVectorOfCameraHeight;
        std::vector< yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* > * ptrVectorOfRgbPortPtr;
        std::vector< yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> >* > * ptrVectorOfIntPortPtr;
        std::vector< yarp::os::BufferedPort<yarp::os::Bottle >* > * ptrVectorOfForce6DPortPtr;
        //
        /** Vector to store pointers to ManipulatorWrapper objects */
        std::vector < ControlboardContainer* > * ptrVectorOfManipulatorWrapperPtr;
};

}  // namespace teo

#endif  // __TEO_COLLISION_AVOIDANCE_RATE_THREAD_HPP__


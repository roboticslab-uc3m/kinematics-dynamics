// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_SIM_HPP__
#define __TEO_SIM_HPP__

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

#include "ColorDebug.hpp"
#include "ManipulatorWrapper.hpp"
#include "TeoSimRateThread.hpp"

#define DEFAULT_ENV "models/teoWhite/teoFull.robot.xml"
#define DEFAULT_JMC_MS 20.0  // [ms]
#define DEFAULT_PHYSICS "none"
#define DEFAULT_VIEWER 1

using namespace std;

using namespace yarp::os;
using namespace yarp::dev;

using namespace OpenRAVE;

namespace teo
{

/**
 *
 * @ingroup TeoSim
 * The TeoSim class creates an instance of OpenRAVE-core (qtcoin viewer included) and implements...
 *
 */
class TeoSim : public RFModule {

    public:

// -------- RFModule declarations. Implementation in TeoSim.cpp --------

        /**
         * Configure the module, pass a ResourceFinder object to the module.
         *
         * @param rf a previously initialized ResourceFinder
         * @return true/false upon success/failure
         *
         * \note attachTerminal() is no longer called automatically. You
         * can call it in the configure function.
         */
        virtual bool configure(yarp::os::ResourceFinder &rf);

        /**
         * You can override this to control the approximate periodicity at which
         * updateModule() is called by runModule().  By default, it returns
         * 1.0. Time here is in seconds.
         *
         * @return the desired period between successive calls to updateModule()
         */
        virtual double getPeriod();

        /**
         * Override this to do whatever your module needs to do.
         *
         * When your module wants to stop, return false.  The module's actual
         * work could be done during this call, or it could just check the
         * state of a thread running in the background.
         *
         * @return true iff module should continue
        */
        virtual bool updateModule();

        /**
         * Close function.
         *
         * This is called automatically when the module closes, after the last call
         * to updateModule.
         * Override this to perform memory cleanup or other activities.
         *
         * @return true/false on success failure.
         */
        virtual bool close();

    // ------------------------------- Protected -------------------------------------

    protected:

        // Rave-specific parameters //
        EnvironmentBasePtr environmentPtr;
        PhysicsEngineBasePtr physicsEnginePtr;
        vector<RobotBasePtr> vectorOfRobotPtr;
        //
        boost::thread_group orThreads;
        //
        std::vector< SensorBasePtr > vectorOfSensorPtrForCameras;
        std::vector< SensorBasePtr > vectorOfSensorPtrForLasers;
        std::vector< boost::shared_ptr<SensorBase::CameraSensorData> > vectorOfCameraSensorDataPtr;
        std::vector< boost::shared_ptr<SensorBase::LaserSensorData> >  vectorOfLaserSensorDataPtr;
        std::vector<int> vectorOfCameraWidth;
        std::vector<int> vectorOfCameraHeight;
        std::vector< BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* > vectorOfRgbPortPtr;
        std::vector< BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> >* > vectorOfIntPortPtr;
        //
        TeoSimRateThread teoSimRateThread;

        /** Vector to store pointers to ManipulatorWrapper objects */
        std::vector < ManipulatorWrapper* > vectorOfManipulatorWrapperPtr;
};

}  // namespace teo

#endif  // __TEO_SIM_HPP__


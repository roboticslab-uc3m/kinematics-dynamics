// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoSim.hpp"

void SetViewer(EnvironmentBasePtr penv, const std::string& viewername, int _viewer);

// ------------------- RFModule Related ------------------------------------

bool teo::TeoSim::configure(yarp::os::ResourceFinder &rf) {

    const double defautTr = M_PI/180.0;
    ConstString env = DEFAULT_ENV;
    double jmcMs = DEFAULT_JMC_MS;
    ConstString physics = DEFAULT_PHYSICS;
    int viewer = DEFAULT_VIEWER;

    if(rf.check("help")) {
        printf("TeoSim options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");

        printf("\t--env [xml] (env in abs, default: \"%s\")\n",env.c_str());
        printf("\t--physics [type] (type of physics, default: \"%s\")\n",physics.c_str());
        printf("\t--viewer [type] (set to 0 for none, default: \"%d\")\n",viewer);
    }

    if (rf.check("env")) env = rf.find("env").asString();
    if (rf.check("jmcMs")) jmcMs = rf.find("jmcMs").asDouble();
    if (rf.check("physics")) physics = rf.find("physics").asString();
    if (rf.check("viewer")) viewer = rf.find("viewer").asInt();

    printf("TeoSim using context: %s.\n",rf.getContextPath().c_str());
    printf("TeoSim using env: %s, jmcMs: %f, physics: %s, viewer: %d.\n", env.c_str(), jmcMs, physics.c_str(), viewer);

    if(rf.check("help")) {
        ::exit(1);
    }

    // Initialize OpenRAVE-core
    RaveInitialize(true);  // Start openrave core
    environmentPtr = RaveCreateEnvironment();  // Create the main OpenRAVE environment, set the EnvironmentBasePtr
    environmentPtr->SetDebugLevel(Level_Debug);  // Relatively new function
    environmentPtr->StopSimulation();  // NEEDED??
    boost::thread thviewer(boost::bind(SetViewer,environmentPtr,"qtcoin",viewer));
    orThreads.add_thread(&thviewer);
    Time::delay(0.4); // wait for the viewer to init, in [s]

    ConstString envFull(rf.getContextPath());
    envFull += "/../models/";
    envFull += env;
    if (!environmentPtr->Load(envFull.c_str())) {
        fprintf(stderr,"[TeoSim] error: Could not load %s environment.\n",envFull.c_str());
        if (!environmentPtr->Load(env.c_str())) {
            fprintf(stderr,"[TeoSim] error: Could not load %s environment.\n",env.c_str());
            return false;
        }
    }
    printf("[TeoSim] success: Loaded environment.\n");

    // Attach a physics engine
    if(physics=="ode"){
        environmentPtr->SetPhysicsEngine(RaveCreatePhysicsEngine(environmentPtr,"ode"));
        environmentPtr->GetPhysicsEngine()->SetGravity(OpenRAVE::Vector(0,0,-9.8));
    }

    //-- Get robots
    environmentPtr->GetRobots(vectorOfRobotPtr);
    //-- For each robot
    for(size_t robotPtrIdx=0;robotPtrIdx<vectorOfRobotPtr.size();robotPtrIdx++) {
        CD_INFO( "Robots[%zu]: %s\n",robotPtrIdx,vectorOfRobotPtr[robotPtrIdx]->GetName().c_str());
        //-- Get manupilators
        vector<RobotBase::ManipulatorPtr> vectorOfManipulatorPtr = vectorOfRobotPtr[robotPtrIdx]->GetManipulators();
        //-- For each manipulator
        for(size_t j=0;j<vectorOfManipulatorPtr.size();j++) {
            CD_INFO( "* Manipulators[%zu]: %s\n",j,vectorOfManipulatorPtr[j]->GetName().c_str() );
            //-- Formulate the manipulator port name
            ConstString manipulatorPortName("/");
            manipulatorPortName += vectorOfRobotPtr[robotPtrIdx]->GetName();
            manipulatorPortName += "/";
            manipulatorPortName += vectorOfManipulatorPtr[j]->GetName();
            CD_INFO( "* manipulatorPortName: %s\n",manipulatorPortName.c_str() );
            //-- Create the manipulator wrapper object
            size_t index = vectorOfManipulatorWrapperPtr.size();
            vectorOfManipulatorWrapperPtr.push_back( new ManipulatorWrapper() );  //! \todo Delete objects stored in vectorOfManipulatorWrapperPtr
            //-- Give it its name
            vectorOfManipulatorWrapperPtr[index]->setManipulatorWrapperName(manipulatorPortName);
            //-- Give it its father's index
            vectorOfManipulatorWrapperPtr[index]->setFatherRobotIdx(robotPtrIdx);
            //-- Check if there are overrides
            if(rf.check(manipulatorPortName)) {
                Bottle manipulatorDescription = rf.findGroup(manipulatorPortName).tail();
                Bottle manipulatorIDs = manipulatorDescription.findGroup("IDs").tail();
                for(int i = 0; i<manipulatorIDs.size(); i++) {
                    vectorOfManipulatorWrapperPtr[index]->push_back( manipulatorIDs.get(i).asInt() );
                }
                CD_INFO( "* Using overriden IDs: %s.\n",manipulatorIDs.toString().c_str());
                if (manipulatorDescription.check("TRs")){
                    Bottle manipulatorTRs = manipulatorDescription.findGroup("TRs").tail();
                    for(int i = 0; i<manipulatorTRs.size(); i++) {
                        vectorOfManipulatorWrapperPtr[index]->push_back_tr( manipulatorTRs.get(i).asDouble() );
                    }
                    CD_INFO( "* Using overriden TRs: %s.\n",manipulatorTRs.toString().c_str());
                } else {
                    for(int i = 0; i<manipulatorIDs.size(); i++) {
                        vectorOfManipulatorWrapperPtr[index]->push_back_tr( defautTr );
                    }CD_INFO( "* Using default general TR: %f.\n", defautTr);
                }
            } else { //-- Use existing IDs otherwise
                std::vector< int > manipulatorIDs = vectorOfManipulatorPtr[j]->GetArmIndices();
                CD_INFO( "* Using xml IDs: ");
                for(size_t i=0;i<manipulatorIDs.size();i++) {
                    printf(" %d", manipulatorIDs[i]);
                    vectorOfManipulatorWrapperPtr[index]->push_back( (int)manipulatorIDs[i] );
                    vectorOfManipulatorWrapperPtr[index]->push_back_tr ( defautTr );
                }
                printf(".\n");
                CD_INFO( "* Using default general TR: %f.\n",defautTr);
            }
            if(!vectorOfManipulatorWrapperPtr[index]->start()){
                CD_ERROR("Could not start ManipulatorWrapper[%zu].\n",index);
                return false;
            }
            //createManipulatorDevice(manipulatorPortName,manipulatorIndices.size());

        }
    }

    for ( unsigned int robotIter = 0; robotIter<vectorOfRobotPtr.size(); robotIter++ ) {
        std::vector<RobotBase::AttachedSensorPtr> sensors = vectorOfRobotPtr.at(robotIter)->GetAttachedSensors();
        printf("Sensors found on robot %d (%s): %d.\n",robotIter,vectorOfRobotPtr.at(robotIter)->GetName().c_str(),(int)sensors.size());
        for ( unsigned int sensorIter = 0; sensorIter<sensors.size(); sensorIter++ ) {
            SensorBasePtr psensorbase = sensors.at(sensorIter)->GetSensor();
            std::string tipo = psensorbase->GetName();
            printf("Sensor %d name: %s\n",sensorIter,tipo.c_str());
            // printf("Sensor %d description: %s\n",sensorIter,psensorbase->GetDescription().c_str());
            if (psensorbase->Supports(SensorBase::ST_Camera)) {
                printf("Sensor %d supports ST_Camera.\n", sensorIter);
                // Activate the camera
                psensorbase->Configure(SensorBase::CC_PowerOn);
                // Show the camera image in a separate window
                // pcamerasensorbase->Configure(SensorBase::CC_RenderDataOn);
                // Get some camera parameter info
                boost::shared_ptr<SensorBase::CameraGeomData> pcamerageomdata = boost::dynamic_pointer_cast<SensorBase::CameraGeomData>(psensorbase->GetSensorGeometry(SensorBase::ST_Camera));
                // printf("Camera width: %d, height: %d.\n",pcamerageomdata->width,pcamerageomdata->height);
                vectorOfCameraWidth.push_back(pcamerageomdata->width);
                vectorOfCameraHeight.push_back(pcamerageomdata->height);
                // Get a pointer to access the camera data stream
                vectorOfCameraSensorDataPtr.push_back(boost::dynamic_pointer_cast<SensorBase::CameraSensorData>(psensorbase->CreateSensorData(SensorBase::ST_Camera)));
                vectorOfSensorPtrForCameras.push_back(psensorbase);  // "save"
                BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* tmpPort = new BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >;
                ConstString tmpName("/");
                ConstString cameraSensorString(psensorbase->GetName());
                size_t pos = cameraSensorString.find("imageMap");
                if ( pos != std::string::npos) {
                    tmpName += cameraSensorString.substr (0,pos-1);
                    tmpName += "/imageMap:o";
                } else {
                    tmpName += cameraSensorString.c_str();
                    tmpName += "/img:o";
                }
                tmpPort->open(tmpName);
                vectorOfRgbPortPtr.push_back(tmpPort);
            } else if (psensorbase->Supports(SensorBase::ST_Laser)) {
                printf("Sensor %d supports ST_Laser.\n", sensorIter);
                // Activate the sensor
                psensorbase->Configure(SensorBase::CC_PowerOn);
                // Paint the rays in the OpenRAVE viewer
                psensorbase->Configure(SensorBase::CC_RenderDataOn);
                // Get some laser parameter info
                // boost::shared_ptr<SensorBase::LaserGeomData> plasergeomdata = boost::dynamic_pointer_cast<SensorBase::LaserGeomData>(psensorbase->GetSensorGeometry(SensorBase::ST_Laser));
                // printf("Laser resolution: %f   %f.\n",plasergeomdata->resolution[0],plasergeomdata->resolution[1]);
                // printf("Laser min_angle: %f   %f.\n",plasergeomdata->min_angle[0],plasergeomdata->min_angle[1]);
                // printf("Laser max_angle: %f   %f.\n",plasergeomdata->max_angle[0],plasergeomdata->max_angle[1]);
                // Get a pointer to access the laser data stream
                vectorOfLaserSensorDataPtr.push_back(boost::dynamic_pointer_cast<SensorBase::LaserSensorData>(psensorbase->CreateSensorData(SensorBase::ST_Laser)));
                vectorOfSensorPtrForLasers.push_back(psensorbase);  // "save"
                BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> >* tmpPort = new BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> >;
                ConstString tmpName("/");
                ConstString depthSensorString(psensorbase->GetName());
                size_t pos = depthSensorString.find("depthMap");
                if ( pos != std::string::npos) {
                    tmpName += depthSensorString.substr (0,pos-1);
                    tmpName += "/depthMap:o";
                } else {
                    tmpName += depthSensorString.c_str();
                    tmpName += "/depth:o";
                }
                tmpPort->open(tmpName);
                vectorOfIntPortPtr.push_back(tmpPort);
            } else printf("Sensor %d not supported.\n", robotIter);
        }
    }

    // Start the RateThread
    teoSimRateThread.setRate(jmcMs);
    teoSimRateThread.setEnvironmentPtr(environmentPtr);
    teoSimRateThread.setPtrVectorOfRobotPtr(&vectorOfRobotPtr);
    teoSimRateThread.setPtrVectorOfManipulatorWrapperPtr(&vectorOfManipulatorWrapperPtr);

    teoSimRateThread.setPtrVectorOfSensorPtrForCameras(&vectorOfSensorPtrForCameras);
    teoSimRateThread.setPtrVectorOfCameraSensorDataPtr(&vectorOfCameraSensorDataPtr);
    teoSimRateThread.setPtrVectorOfRgbPortPtr(&vectorOfRgbPortPtr);
    teoSimRateThread.setPtrVectorOfIntPortPtr(&vectorOfIntPortPtr);
    teoSimRateThread.setPtrVectorOfCameraWidth(&vectorOfCameraWidth);
    teoSimRateThread.setPtrVectorOfCameraHeight(&vectorOfCameraHeight);
    teoSimRateThread.setPtrVectorOfSensorPtrForLasers(&vectorOfSensorPtrForLasers);
    teoSimRateThread.setPtrVectorOfLaserSensorDataPtr(&vectorOfLaserSensorDataPtr);

    teoSimRateThread.start();
    
    return true;
}

// -----------------------------------------------------------------------------

double teo::TeoSim::getPeriod() {
    return 5.0;
}

// -----------------------------------------------------------------------------

bool teo::TeoSim::updateModule() {
    CD_INFO("Alive...\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::TeoSim::close() {
    printf("[TeoSim] begin: close(). Ask thread to stop...\n");
    teoSimRateThread.askToStop();
    printf("[TeoSim] Done. Closing devices...\n");
    for (size_t i=0;i<vectorOfManipulatorWrapperPtr.size();i++) {
        vectorOfManipulatorWrapperPtr[i]->stop();
        delete vectorOfManipulatorWrapperPtr[i];
        vectorOfManipulatorWrapperPtr[i] = 0;
    }
    //penv->StopSimulation();  // NEEDED??
    printf("[TeoSim] Devices closed. Closing environment...\n");
    environmentPtr->Destroy(); // destroy
    Time::delay(0.4);
    printf("[TeoSim] close() joining...\n");
    orThreads.join_all(); // wait for the viewer thread to exit
    printf("[TeoSim] success: join_all ended.\n");
    printf("[TeoSim] end: close().\n");
    return true;
}

// -----------------------------------------------------------------------------

void SetViewer(EnvironmentBasePtr penv, const std::string& viewername, int _viewer)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AddViewer(viewer);  // penv->AttachViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true; // change to false to disable scene view
    if(!_viewer) showgui = false;  // if viewer arg = 0
    viewer->main(showgui);
}

// -----------------------------------------------------------------------------

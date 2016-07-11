// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoSimRateThread.hpp"

// ------------------- RateThread Related ------------------------------------

bool teo::TeoSimRateThread::threadInit() {
    printf("[TeoSimRateThread] begin: threadInit()\n");
    jmcMs = this->getRate();

    yarp::os::ConstString externObj = DEFAULT_EXTERN_OBJ;

    if(externObj=="redCan") {
        OpenRAVE::RaveLoadPlugin("ExternObj");
        OpenRAVE::ModuleBasePtr pExternObj = RaveCreateModule(environmentPtr,"ExternObj"); // create the module
        environmentPtr->Add(pExternObj,true); // load the module, calls main and also enables good destroy.
        std::stringstream cmdin,cmdout;
        cmdin << "Open";  // default maxiter:4000
        RAVELOG_INFO("%s\n",cmdin.str().c_str());
        if( !pExternObj->SendCommand(cmdout,cmdin) ) {
            fprintf(stderr,"Bad send Open command.\n");
        }
        printf("Sent Open command.\n");
    }

    lastTime = yarp::os::Time::now();
    printf("[TeoSimRateThread] end: threadInit()\n");
    return true;
}

// -----------------------------------------------------------------------------

void teo::TeoSimRateThread::run() {
    //printf("[TeoSimRateThread] run()\n");

    for(size_t i=0;i<ptrVectorOfRobotPtr->size();i++) {  // For each robot
        int dof = ptrVectorOfRobotPtr->at(i)->GetDOF();  // Create a vector sized
        std::vector<OpenRAVE::dReal> dEncRaw(dof);                 // its number of joints.
        //-- Iterate through manipulator wrappers
        for(size_t j=0;j<ptrVectorOfManipulatorWrapperPtr->size();j++) {
            //-- If indexes belong to father
            if((int)i == ptrVectorOfManipulatorWrapperPtr->at(j)->getFatherRobotIdx() ) {
                std::vector< int > vectorOfJointIdx = ptrVectorOfManipulatorWrapperPtr->at(j)->getVectorOfJointIdxRef();
                std::vector< double > vectorOfJointPos = ptrVectorOfManipulatorWrapperPtr->at(j)->getVectorOfJointPosRef();
                std::vector< double > vectorOfJointTr = ptrVectorOfManipulatorWrapperPtr->at(j)->getVectorOfJointTrRef();
                //-- Overwrite dEncRaw
                for(size_t k=0;k<vectorOfJointIdx.size();k++) {
                    dEncRaw[vectorOfJointIdx[k]] = (vectorOfJointPos[k]) * (vectorOfJointTr[k]);
                }
            }
        }
        ptrVectorOfRobotPtr->at(i)->SetJointValues(dEncRaw);  // More compatible with physics??
    }

    environmentPtr->StepSimulation(jmcMs/1000.0);  // StepSimulation must be given in seconds

    for(unsigned int camIter = 0; camIter<ptrVectorOfSensorPtrForCameras->size(); camIter++ ) {
        ptrVectorOfSensorPtrForCameras->at(camIter)->GetSensorData(ptrVectorOfCameraSensorDataPtr->at(camIter));
        //std::vector<uint8_t> currentFrame = pcamerasensordata->vimagedata;
        //printf("Vector size: %d\n",currentFrame.size()); // i.e. 480 * 640 * 3 = 921600;
        yarp::sig::ImageOf<yarp::sig::PixelRgb>& i_imagen = ptrVectorOfRgbPortPtr->at(camIter)->prepare();
        i_imagen.resize(ptrVectorOfCameraWidth->at(camIter),ptrVectorOfCameraHeight->at(camIter));  // Tamaño de la pantalla
        yarp::sig::PixelRgb p;
        for (int i_x = 0; i_x < i_imagen.width(); ++i_x) {
            for (int i_y = 0; i_y < i_imagen.height(); ++i_y) {
                p.r = ptrVectorOfCameraSensorDataPtr->at(camIter)->vimagedata[3*(i_x+(i_y*i_imagen.width()))];
                p.g = ptrVectorOfCameraSensorDataPtr->at(camIter)->vimagedata[1+3*(i_x+(i_y*i_imagen.width()))];
                p.b = ptrVectorOfCameraSensorDataPtr->at(camIter)->vimagedata[2+3*(i_x+(i_y*i_imagen.width()))];
                i_imagen.safePixel(i_x,i_y) = p;
            }
        }
        ptrVectorOfRgbPortPtr->at(camIter)->write();
    }
    
    for(unsigned int laserIter = 0; laserIter<ptrVectorOfSensorPtrForLasers->size(); laserIter++ ) {
        ptrVectorOfSensorPtrForLasers->at(laserIter)->GetSensorData(ptrVectorOfLaserSensorDataPtr->at(laserIter));
        std::vector< OpenRAVE::RaveVector< OpenRAVE::dReal > > sensorRanges = ptrVectorOfLaserSensorDataPtr->at(laserIter)->ranges;
        std::vector< OpenRAVE::RaveVector< OpenRAVE::dReal > > sensorPositions = ptrVectorOfLaserSensorDataPtr->at(laserIter)->positions;
        OpenRAVE::Transform tinv = ptrVectorOfLaserSensorDataPtr->at(laserIter)->__trans.inverse();
        // std::vector< dReal > sensorIntensity = plasersensordata[laserIter]->intensity;
        // printf("[%d] sensorPositions size: %d ",laserIter,sensorPositions.size()); // = 1; xyz of the fixed 3d sensor position.
        // printf("sensorRanges size: %d ",sensorRanges.size()); // 64 * 48 = 3072;
        // printf("sensorIntensity size: %d\n",sensorIntensity.size()); // 64 * 48 = 3072;
        //for(unsigned int i=0;i<sensorRanges.size();i++) {
        //   printf("sensorRanges[%d].x: %f  sensorRanges[%d].y: %f  sensorRanges[%d].z: %f sensorIntensity[%d]: %.2f\n",i,sensorRanges[i].x,i,sensorRanges[i].y,i,sensorRanges[i].z,i,sensorIntensity[i]);  // intensity always 1.0 or 0.0 in 3d sensor
        //}*/
        /*Transform tinv = plasersensordata->__trans.inverse();
        for(size_t i = 0; i < _data->ranges.size(); ++i) {
            float* p = (float*)(&_pointcloud2msg.data.at(i*_pointcloud2msg.point_step));
            if( i < _data->positions.size() ) {
                Vector v = tinv*(_data->ranges[i] + _data->positions[i]);
                p[0] = (float)v.x;
                p[1] = (float)v.y;
                p[2] = (float)v.z;
            } else if( _data->positions.size() > 0 ) {
                Vector v = tinv*(_data->ranges[i] + _data->positions[0]);
                p[0] = (float)v.x;
                p[1] = (float)v.y;
                p[2] = (float)v.z;
            } else {
                Vector v = tinv*_data->ranges[i];
                p[0] = (float)v.x;
                p[1] = (float)v.y;
                p[2] = (float)v.z;
            }
            if( _data->intensity.size()==_data->ranges.size() ) {
                p[3] = _data->intensity[i];
            }
        }*/
        yarp::sig::ImageOf<yarp::sig::PixelInt>& i_depth = ptrVectorOfIntPortPtr->at(laserIter)->prepare();
        if(sensorRanges.size()==3072) i_depth.resize(64,48);  // Tamaño de la pantalla (64,48)
        else if(sensorRanges.size()==12288) i_depth.resize(128,96);
        else if(sensorRanges.size()==49152) i_depth.resize(256,192);
        else if(sensorRanges.size()==307200) i_depth.resize(640,480);
        else if(sensorRanges.size()==4) i_depth.resize(2,2);
        //else printf("[warning] unrecognized laser sensor data size.\n");
        else i_depth.resize(sensorRanges.size(),1);
        for (int i_y = 0; i_y < i_depth.height(); ++i_y) {  // was y in x before
            for (int i_x = 0; i_x < i_depth.width(); ++i_x) {
                //double p = sensorRanges[i_y+(i_x*i_depth.height())].z;
                double p;
                if( sensorPositions.size() > 0 ) {
                    OpenRAVE::Vector v = tinv*(sensorRanges[i_y+(i_x*i_depth.height())] + sensorPositions[0]);
                    p = (float)v.z;
                } else {
                    OpenRAVE::Vector v = tinv*(sensorRanges[i_y+(i_x*i_depth.height())]);
                    p = (float)v.z;
                }
                i_depth(i_x,i_y) = p*1000.0;  // give mm
            }
        }
        ptrVectorOfIntPortPtr->at(laserIter)->write();
    }

    for(unsigned int force6DIter = 0; force6DIter<ptrVectorOfSensorPtrForForce6Ds->size(); force6DIter++ ) {
        ptrVectorOfSensorPtrForForce6Ds->at(force6DIter)->GetSensorData(ptrVectorOfForce6DSensorDataPtr->at(force6DIter));
        yarp::os::Bottle& b = ptrVectorOfForce6DPortPtr->at(force6DIter)->prepare();
        b.clear();
        b.addDouble( ptrVectorOfForce6DSensorDataPtr->at(force6DIter)->force[0] );
        b.addDouble( ptrVectorOfForce6DSensorDataPtr->at(force6DIter)->force[1] );
        b.addDouble( ptrVectorOfForce6DSensorDataPtr->at(force6DIter)->force[2] );
        b.addDouble( ptrVectorOfForce6DSensorDataPtr->at(force6DIter)->torque[0] );
        b.addDouble( ptrVectorOfForce6DSensorDataPtr->at(force6DIter)->torque[1] );
        b.addDouble( ptrVectorOfForce6DSensorDataPtr->at(force6DIter)->torque[2] );
        ptrVectorOfForce6DPortPtr->at(force6DIter)->write();
    }

}

// -----------------------------------------------------------------------------


// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ManipulatorWrapper.hpp"

// -----------------------------------------------------------------------------

const int teo::ManipulatorWrapper::getFatherRobotIdx() {
    return fatherRobotIdx;
}

// -----------------------------------------------------------------------------

 std::vector<int>& teo::ManipulatorWrapper::getVectorOfJointIdxRef() {
    return vectorOfJointIdx;
}

 // -----------------------------------------------------------------------------

  std::vector<double>& teo::ManipulatorWrapper::getVectorOfJointTrRef() {
     return vectorOfJointTr;
 }

// -----------------------------------------------------------------------------

std::vector<double>& teo::ManipulatorWrapper::getVectorOfJointPosRef() {
    if(encs) {
        double vals[vectorOfJointIdx.size()];
        encs->getEncoders(vals);
        for(size_t i=0; i < this->vectorOfJointIdx.size(); i++)
            vectorOfJointPos[i] = vals[i];
    } else {
        CD_WARNING("No encs yet.\n")
    }
    return vectorOfJointPos;
}
// -----------------------------------------------------------------------------

void teo::ManipulatorWrapper::setFatherRobotIdx(int value) {
    fatherRobotIdx = value;
}

// -----------------------------------------------------------------------------

void teo::ManipulatorWrapper::setManipulatorWrapperName(const std::string &value) {
    manipulatorWrapperName = value;
}

// -----------------------------------------------------------------------------

void teo::ManipulatorWrapper::push_back(int robotJointIdx) {
    vectorOfJointIdx.push_back( robotJointIdx );
}

// -----------------------------------------------------------------------------

void teo::ManipulatorWrapper::push_back_tr(double robotJointTr) {
    vectorOfJointTr.push_back( robotJointTr );
}

// -----------------------------------------------------------------------------

bool teo::ManipulatorWrapper::start() {
    vectorOfJointPos.resize( this->vectorOfJointIdx.size() );
    Property options;
    options.put("device","controlboardwrapper2");  // was controlboard, now this for ravepart2
    options.put("subdevice","ravepart2");  // ravepart provides more interfaces than test_motor
    options.put("axes", (int)this->vectorOfJointIdx.size() );
    options.put("name", this->manipulatorWrapperName );
    dd.open(options);
    if(!dd.isValid()) {
        CD_ERROR("ManipulatorWrapper device \"%s\" not available.\n", options.find("subdevice").asString().c_str());
        dd.close();
        return false;
    }
    dd.view(encs);
    return true;
}

// -----------------------------------------------------------------------------

bool teo::ManipulatorWrapper::stop() {
    dd.close();
    return true;
}

// -----------------------------------------------------------------------------

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ControlboardContainer.hpp"

// -----------------------------------------------------------------------------

const int teo::ControlboardContainer::getFatherRobotIdx() {
    return fatherRobotIdx;
}

// -----------------------------------------------------------------------------

 std::vector<int>& teo::ControlboardContainer::getVectorOfJointIdxRef() {
    return vectorOfJointIdx;
}

 // -----------------------------------------------------------------------------

  std::vector<double>& teo::ControlboardContainer::getVectorOfJointTrRef() {
     return vectorOfJointTr;
 }

// -----------------------------------------------------------------------------

std::vector<double>& teo::ControlboardContainer::getVectorOfJointPosRef() {
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

void teo::ControlboardContainer::setFatherRobotIdx(int value) {
    fatherRobotIdx = value;
}

// -----------------------------------------------------------------------------

void teo::ControlboardContainer::setManipulatorWrapperName(const std::string &value) {
    manipulatorWrapperName = value;
}

// -----------------------------------------------------------------------------

void teo::ControlboardContainer::push_back(int robotJointIdx) {
    vectorOfJointIdx.push_back( robotJointIdx );
}

// -----------------------------------------------------------------------------

void teo::ControlboardContainer::push_back_tr(double robotJointTr) {
    vectorOfJointTr.push_back( robotJointTr );
}

// -----------------------------------------------------------------------------

bool teo::ControlboardContainer::start() {
    vectorOfJointPos.resize( this->vectorOfJointIdx.size() );
    yarp::os::Property options;
    options.put("device","controlboardwrapper2");  //
    options.put("subdevice","FakeControlboard");  // FakeControlboard provides more interfaces than test_motor
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

bool teo::ControlboardContainer::stop() {
    dd.close();
    return true;
}

// -----------------------------------------------------------------------------

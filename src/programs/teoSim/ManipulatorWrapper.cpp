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
    /*Property options;
    options.put("device","controlboard");  //
    options.put("subdevice","ravepart2");  // ravepart provides more interfaces than test_motor
    options.put("axes", (int)this->vectorOfJointIdx.size() );
    options.put("name", this->manipulatorWrapperName );
    dd.open(options);
    if(!dd.isValid()) {
        CD_ERROR("ManipulatorWrapper device \"%s\" not available.\n", options.find("subdevice").asString().c_str());
        dd.close();
        return false;
    }
    dd.view(encs);*/

    Property parameters;
    parameters.put("device", "ravepart");
    parameters.put("axes", (int)this->vectorOfJointIdx.size() );
    dd.open(parameters);
    if(!dd.isValid()) {
        CD_ERROR("ManipulatorWrapper device \"%s\" not available.\n", parameters.find("device").asString().c_str());
        dd.close();
        return false;
    }
    dd.view(encs);

    PolyDriver wrapperHead;
    Property paramsHead;
    paramsHead.put("device", "controlboardwrapper2");
    paramsHead.put("name", this->manipulatorWrapperName );
    Value tmp;
    tmp.fromString("(ravepart)");
    paramsHead.put("joints", 3);
    paramsHead.put("networks", tmp);
    //map joints 0-2 from fakebot to 0-2 of part robot/head
    tmp.fromString("(0 2 0 2)");
    paramsHead.put("ravepart", tmp);

    IMultipleWrapper *iwrapperHead;
    wrapperHead.view(iwrapperHead);
    PolyDriverList list;
    list.push(&dd, "ravepart");
    iwrapperHead->attachAll(list);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::ManipulatorWrapper::stop() {
    dd.close();
    return true;
}

// -----------------------------------------------------------------------------

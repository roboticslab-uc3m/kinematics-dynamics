// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoXRpcResponder.hpp"

namespace teo
{

/************************************************************************/

bool TeoXRpcResponder::read(ConnectionReader& connection) {
    Bottle in, out;
    in.read(connection);
    printf("[xRpcResponder] Got %s\n", in.toString().c_str());
    out.clear();
    ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender==NULL) return false;
    if ((in.get(0).asString() == "help")||(in.get(0).asVocab() == VOCAB_HELP)) {  // help //
        out.addString("Available commands: [help] [inv] [movj] [movl] [stat] [stop] [wait] [tool] more info at [http://roboticslab.sourceforge.net/asibot/group__cartesianServer.html]");
        return out.write(*returnToSender);
    } else if ((in.get(0).asString() == "stop")||(in.get(0).asVocab() == VOCAB_STOP)) {  // stop //
        if(icart->stopControl()) {
            if(ipos->stop()) {
                if(ipos->setPositionMode()) {
                    out.addVocab(VOCAB_OK);
                    *csStatus = 0;
                } else out.addVocab(VOCAB_FAILED);
            } else out.addVocab(VOCAB_FAILED);
        } else out.addVocab(VOCAB_FAILED);
        return out.write(*returnToSender);
    } else if ((in.get(0).asString() == "wait")||(in.get(0).asVocab() == VOCAB_WAIT)) {  // wait //
        while (*csStatus != 0) {
            printf(".");
            fflush(stdout);
            if (*csStatus==1) { // movj
                bool done;
                ipos->checkMotionDone(&done);
                if(done) *csStatus = 0;
            } else if (*csStatus==2) { // movl
                bool done;
                icart->checkMotionDone(&done);
                if(done) *csStatus = 0;
            }
            Time::delay(0.5);
        }
        printf("\n");
        out.addVocab(VOCAB_OK);
        return out.write(*returnToSender);
    } else if ((in.get(0).asString() == "stat")||(in.get(0).asVocab() == VOCAB_STAT)) { // stat //
        if (*csStatus==1) { // movj
            bool done;
            ipos->checkMotionDone(&done);
            if(done) *csStatus = 0;
        } else if (*csStatus==2) { // movl
            bool done;
            icart->checkMotionDone(&done);
            if(done) *csStatus = 0;
        }        
        if (*csStatus==0) out.addVocab(VOCAB_MY_STOP);
        else if (*csStatus==1) out.addVocab(VOCAB_MOVJ);
        else if (*csStatus==2) out.addVocab(VOCAB_MOVL);
        else out.addVocab(VOCAB_FAILED);
        Vector x,o;
        if(icart->getPose(x,o)) {
            Bottle& l1 = out.addList();
            for (int i = 0; i < x.size(); i++)
                l1.addDouble(x[i]);
            for (int i = 0; i < o.size(); i++)
                l1.addDouble(o[i]);
            out.addVocab(VOCAB_OK);
        } else
            out.addVocab(VOCAB_FAILED);
        return out.write(*returnToSender);
    } else if ((in.get(0).asString() == "movl")||(in.get(0).asVocab() == VOCAB_MOVL)) { // movl //
        Vector x,o;
        Bottle *lst = in.get(1).asList();
        x.push_back(lst->get(0).asDouble());
        x.push_back(lst->get(1).asDouble());
        x.push_back(lst->get(2).asDouble());
        for (int i = 3; i < lst->size(); i++)
            o.push_back(lst->get(i).asDouble());
        if(icart->goToPoseSync(x,o)){
            *csStatus = 2;
            out.addVocab(VOCAB_OK);
        } else
            out.addVocab(VOCAB_FAILED);
        return out.write(*returnToSender);
    } else if ((in.get(0).asString() == "movj")||(in.get(0).asVocab() == VOCAB_MOVJ)) { // movj //
        Vector xd,od,xdhat,odhat,qdhat;
        Bottle *lst = in.get(1).asList();
        xd.push_back(lst->get(0).asDouble());
        xd.push_back(lst->get(1).asDouble());
        xd.push_back(lst->get(2).asDouble());
        for (int i = 3; i < lst->size(); i++)
            od.push_back(lst->get(i).asDouble());
        if(icart->askForPose(xd,od,xdhat,odhat,qdhat)){
            //double qd[qdhat.size()];
            double qd[MAX_NUM_MOTORS];  // should actually do a malloc depending on qdhat.size() 
            for (int i = 0; i < qdhat.size(); i++)
                qd[i] = qdhat[i];
            icart->stopControl(); // new!!!!
            ipos->setPositionMode();
            ipos->positionMove(qd);
            *csStatus = 1;
            out.addVocab(VOCAB_OK);
        } else
            out.addVocab(VOCAB_FAILED);
        return out.write(*returnToSender);
    } else if ((in.get(0).asString() == "inv")||(in.get(0).asVocab() == VOCAB_INV)) { // inv //
        Vector xd,od,xdhat,odhat,qdhat;
        Bottle *lst = in.get(1).asList();
        xd.push_back(lst->get(0).asDouble());
        xd.push_back(lst->get(1).asDouble());
        xd.push_back(lst->get(2).asDouble());
        for (int i = 3; i < lst->size(); i++)
            od.push_back(lst->get(i).asDouble());
        if(icart->askForPose(xd,od,xdhat,odhat,qdhat)){
            Bottle& l1 = out.addList();
            for (int i = 0; i < qdhat.size(); i++)
                l1.addDouble(qdhat[i]);
            out.addVocab(VOCAB_OK);
        } else
            out.addVocab(VOCAB_FAILED);
        return out.write(*returnToSender);
    } else if ((in.get(0).asString() == "tool")||(in.get(0).asVocab() == VOCAB_TOOL)) { // tool //
        int tool = in.get(1).asInt();
        printf("[xRpcResponder] Tool set to: %d\n", tool);
        Bottle tweek;
        tweek.addString("tool");
        tweek.addInt(tool);
        icart->tweakSet(tweek);
        out.addVocab(VOCAB_OK);
        return out.write(*returnToSender);
    } else {
        fprintf(stderr,"[xRpcResponder] fail: Unknown command (use 'help' if needed).\n");
        out.addVocab(VOCAB_FAILED);
        return out.write(*returnToSender);
    }
}

/************************************************************************/

void TeoXRpcResponder::setCartesianInterface(yarp::dev::ICartesianControl* _icart) {
    icart = _icart;
}

/************************************************************************/

void TeoXRpcResponder::setPositionInterface(yarp::dev::IPositionControl* _ipos) {
    ipos = _ipos;
}

/************************************************************************/

void TeoXRpcResponder::setCsStatus(int* _csStatus) {
    csStatus = _csStatus;
}

/************************************************************************/

}  // namespace teo

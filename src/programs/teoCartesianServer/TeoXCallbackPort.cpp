
#include "TeoXCallbackPort.hpp"

/************************************************************************/

void TeoXCallbackPort::onRead(Bottle& b) {
    printf("[CartesianServerLib] xCallbackPort Got %s\n", b.toString().c_str());
    if (b.get(0).getCode() == BOTTLE_TAG_DOUBLE) {
        Vector xd,od,xdhat,odhat,qdhat;
        xd.push_back(b.get(0).asDouble());
        xd.push_back(b.get(1).asDouble());
        xd.push_back(b.get(2).asDouble());
        //for (int i = 3; i < lst->size(); i++)
        //    od.push_back(lst->get(i).asDouble());
        if(!icart->askForPose(xd,od,xdhat,odhat,qdhat)) return;
        double qd[qdhat.size()];  // should actually do a malloc depending on qdhat.size() 
        for (int i = 0; i < qdhat.size(); i++)
            qd[i] = qdhat[i];
        //icart->stopControl(); // new!!!!
        ipos->setPositionMode();
        ipos->positionMove(qd);
    }
    int choice = b.get(0).asVocab();
    if (b.get(0).getCode() != BOTTLE_TAG_VOCAB) choice = VOCAB_FAILED;
    if (choice==VOCAB_MY_STOP) {  ///////////////////////////////// stop /////////////////////////////////
        icart->stopControl();
        ipos->stop();
        ipos->setPositionMode();
        *csStatus = 0;
    } else if (choice==VOCAB_FWD) { ///////////////////////////////// fwd /////////////////////////////////
        Vector cmd;
        Bottle *lst = b.get(1).asList();
        cmd.push_back(lst->get(0).asDouble());
        cmd.push_back(lst->get(1).asDouble());
        Vector x,o,xd,od;
        if(!icart->getPose(x,o)) return;
        double ozDeg = (atan2(x[1],x[0]))*180.0/M_PI;
        double PrP = sqrt(x[0]*x[0]+x[1]*x[1]);
        double PrPd = PrP + VPOINT_DIST*sin((cmd[1])*M_PI/180.0);
        //printf("[CartesianServerLib] PrP: %f;PrPd: %f\n",PrP,PrPd);
        double PhPd = x[2] + VPOINT_DIST*cos((cmd[1])*M_PI/180.0);
        xd.push_back(PrPd*cos((ozDeg-cmd[0])*M_PI/180.0));  // xd
        xd.push_back(PrPd*sin((ozDeg-cmd[0])*M_PI/180.0));  // yd
        xd.push_back(PhPd);  // zd
        od.push_back(cmd[1]); // rot(y')d
        od.push_back(0.0); // rot(z'')d
        //printf("[CartesianServerLib] xd: %s; od: %s\n",xd.toString().c_str(),od.toString().c_str());
        *csStatus = 3;
        icart->goToPose(xd,od,3.0);
    } else if (choice==VOCAB_BKWD) { ///////////////////////////////// bkwd /////////////////////////////////
        Vector cmd;
        Bottle *lst = b.get(1).asList();
        cmd.push_back(lst->get(0).asDouble());
        cmd.push_back(lst->get(1).asDouble());
        Vector x,o,xd,od;
        if(!icart->getPose(x,o)) {
            fprintf(stderr,"[CartesianServerLib] warning: getPose failed!\n");
            return;
        }
        double ozDeg = (atan2(x[1],x[0]))*180.0/M_PI;
        double PrP = sqrt(x[0]*x[0]+x[1]*x[1]);
        double PrPd = PrP - VPOINT_DIST*sin((cmd[1])*M_PI/180.0); 
        if (PrPd<=0) {
            printf("[CartesianServerLib] bwd not reachable\n");
            return;
        }
        printf("PrP: %f;PrPd: %f\n",PrP,PrPd);
        double PhPd = x[2] - VPOINT_DIST*cos((cmd[1])*M_PI/180.0);
        xd.push_back(PrPd*cos((ozDeg-cmd[0])*M_PI/180.0));  // xd
        xd.push_back(PrPd*sin((ozDeg-cmd[0])*M_PI/180.0));  // yd
        xd.push_back(PhPd);  // zd
        od.push_back(cmd[1]); // rot(y')d
        od.push_back(0.0); // rot(z'')d
        printf("xd: %s; od: %s\n",xd.toString().c_str(),od.toString().c_str());
        *csStatus = 4;
        icart->goToPose(xd,od,3.0);
    } else if (choice==VOCAB_ROT) { ///////////////////////////////// rot /////////////////////////////////
        Vector cmd;
        Bottle *lst = b.get(1).asList();
        cmd.push_back(lst->get(0).asDouble());
        cmd.push_back(lst->get(1).asDouble());
        Vector x,o,xd,od;
        if(!icart->getPose(x,o)) {
            fprintf(stderr,"[CartesianServerLib] warning: getPose failed!\n");
            return;
        }
        double ozDeg = (atan2(x[1],x[0]))*180.0/M_PI;
        double PrP = sqrt(x[0]*x[0]+x[1]*x[1]);
        double PrPd = PrP;
        printf("PrP: %f;PrPd: %f\n",PrP,PrPd);
        double PhPd = x[2];
        xd.push_back(PrPd*cos((ozDeg-cmd[0])*M_PI/180.0));  // xd
        xd.push_back(PrPd*sin((ozDeg-cmd[0])*M_PI/180.0));  // yd
        xd.push_back(PhPd);  // zd
        od.push_back(cmd[1]); // rot(y')d
        od.push_back(0.0); // rot(z'')d
        printf("xd: %s; od: %s\n",xd.toString().c_str(),od.toString().c_str());
        *csStatus = 5;
        icart->goToPose(xd,od,3.0);
    } else if (choice==VOCAB_VMOS) { ///////////////////////////////// vmos /////////////////////////////////
        Bottle *lst = b.get(1).asList();
        Vector xdotd,odotd;
        xdotd.push_back(lst->get(0).asDouble());
        xdotd.push_back(lst->get(1).asDouble());
        xdotd.push_back(lst->get(2).asDouble());
        for (int i = 3; i < lst->size(); i++)
            odotd.push_back(lst->get(i).asDouble());
        printf("[xCallbackPort] [vmos] xdotd: %s; odotd: %s\n",xdotd.toString().c_str(),odotd.toString().c_str());
        *csStatus = 6;
        icart->setTaskVelocities(xdotd,odotd);
    } else if (choice==VOCAB_POSE) { ///////////////////////////////// pose /////////////////////////////////
        Vector cmd;
        Bottle *lst = b.get(1).asList();
        Vector xd,od;
        xd.push_back(lst->get(0).asDouble());  // xd
        xd.push_back(lst->get(1).asDouble());  // xd
        xd.push_back(lst->get(2).asDouble());  // xd
        od.push_back(lst->get(3).asDouble());  // rot(y')d
        od.push_back(lst->get(4).asDouble());  // rot(z'')d
        //printf("xd: %s; od: %s\n",xd.toString().c_str(),od.toString().c_str());
        *csStatus = 7;
        icart->goToPose(xd,od,3.0);
    }
}

/************************************************************************/

void TeoXCallbackPort::setCartesianInterface(yarp::dev::ICartesianControl* _icart) {
    icart = _icart;
}

/************************************************************************/

void TeoXCallbackPort::setPositionInterface(yarp::dev::IPositionControl* _ipos) {
    ipos = _ipos;
}

/************************************************************************/

void TeoXCallbackPort::setCsStatus(int* _csStatus) {
    csStatus = _csStatus;
}

/************************************************************************/


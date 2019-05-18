// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TwoLimbCartesianControlServer.hpp"

#include <ColorDebug.h>

// ------------------- TwoLimbCartesianControlServer Related ------------------------------------

bool roboticslab::TwoLimbCartesianControlServer::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle in, out;
    bool ok = in.read(connection);
    if (!ok) return false;

    // process data "in", prepare "out"
    CD_DEBUG("Got: %s\n",in.toString().c_str());

    if( in.get(0).asString() == "help")
    {
        out.addVocab(VOCAB_CC_STAT);
        out.addVocab(VOCAB_CC_STEP);
        out.addVocab(VOCAB_CC_STOP);
    }
    else if( in.get(0).asVocab() == VOCAB_CC_STAT)
    {
        std::vector<double> x;
        int state;
        bool ok = iTwoLimbCartesianControl->stat( state, x );
        if(ok)
        {
            out.addVocab(state);
            for(size_t i=0; i<x.size(); i++)
                out.addFloat64(x[i]);
        }
        else
        {
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_STEP)
    {
        bool ok = iTwoLimbCartesianControl->step();
        if(ok)
        {
            out.addVocab(VOCAB_OK);
        }
        else
        {
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_STOP)
    {
        bool ok = iTwoLimbCartesianControl->stopControl();
        if(ok)
        {
            out.addVocab(VOCAB_OK);
        }
        else
        {
            out.addVocab(VOCAB_FAILED);
        }
    }
    else
    {
        out.addVocab(VOCAB_FAILED);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender!=NULL) {
        out.write(*returnToSender);
    }
    return true;
}

// -----------------------------------------------------------------------------

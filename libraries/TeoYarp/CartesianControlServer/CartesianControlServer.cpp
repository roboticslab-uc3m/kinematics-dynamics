// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

// ------------------- CartesianControlServer Related ------------------------------------

bool teo::CartesianControlServer::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle in, out;
    bool ok = in.read(connection);
    if (!ok) return false;

    // process data "in", prepare "out"
    CD_DEBUG("Got: %s\n",in.toString().c_str());

    if( in.get(0).asString() == "help")
    {
        out.addVocab(VOCAB_STAT);
    }
    else if( in.get(0).asVocab() == VOCAB_STAT)
    {
        std::vector<double> x;
        iCartesianControl->stat( x );
        for(size_t i=0; i<x.size(); i++)
            out.addDouble(x[i]);
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

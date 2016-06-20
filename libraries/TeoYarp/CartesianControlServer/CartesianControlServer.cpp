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
        out.addVocab(VOCAB_INV);
    }
    else if( in.get(0).asVocab() == VOCAB_STAT)
    {
        std::vector<double> x;
        iCartesianControl->stat( x );
        for(size_t i=0; i<x.size(); i++)
            out.addDouble(x[i]);
    }
    else if( in.get(0).asVocab() == VOCAB_INV)
    {
        if(in.size()>1)
        {
            std::vector<double> xd,q;
            for(size_t i=1; i<in.size();i++)
                xd.push_back(in.get(i).asDouble());
            bool ok = iCartesianControl->inv(xd,q);
            if(ok)
            {
                for(size_t i=0; i<q.size(); i++)
                    out.addDouble(q[i]);
            }
            else
            {
                out.addVocab(VOCAB_FAILED);
            }
        }
        else
        {
            CD_ERROR("size error\n");
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

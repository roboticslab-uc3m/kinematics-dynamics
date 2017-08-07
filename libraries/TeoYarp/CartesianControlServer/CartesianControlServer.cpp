// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <ColorDebug.hpp>

// ------------------- CartesianControlServer Related ------------------------------------

bool roboticslab::CartesianControlServer::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle in, out;
    bool ok = in.read(connection);
    if (!ok) return false;

    // process data "in", prepare "out"
    CD_DEBUG("Got: %s\n",in.toString().c_str());

    if( in.get(0).asString() == "help")
    {
        out.addVocab(VOCAB_CC_STAT);
        out.addVocab(VOCAB_CC_INV);
        out.addVocab(VOCAB_CC_MOVJ);
        out.addVocab(VOCAB_CC_RELJ);
        out.addVocab(VOCAB_CC_MOVL);
        out.addVocab(VOCAB_CC_MOVV);
        out.addVocab(VOCAB_CC_GCMP);
        out.addVocab(VOCAB_CC_FORC);
        out.addVocab(VOCAB_CC_STOP);
    }
    else if( in.get(0).asVocab() == VOCAB_CC_STAT)
    {
        std::vector<double> x;
        int state;
        bool ok = iCartesianControl->stat( state, x );
        if(ok)
        {
            out.addVocab(state);
            for(size_t i=0; i<x.size(); i++)
                out.addDouble(x[i]);
        }
        else
        {
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_INV)
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
    else if( in.get(0).asVocab() == VOCAB_CC_MOVJ)
    {
        if(in.size()>1)
        {
            std::vector<double> xd;
            for(size_t i=1; i<in.size();i++)
                xd.push_back(in.get(i).asDouble());
            bool ok = iCartesianControl->movj(xd);
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
            CD_ERROR("size error\n");
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_RELJ)
    {
        if(in.size()>1)
        {
            std::vector<double> xd;
            for(size_t i=1; i<in.size();i++)
                xd.push_back(in.get(i).asDouble());
            bool ok = iCartesianControl->relj(xd);
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
            CD_ERROR("size error\n");
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_MOVL)
    {
        if(in.size()>1)
        {
            std::vector<double> xd;
            for(size_t i=1; i<in.size();i++)
                xd.push_back(in.get(i).asDouble());
            bool ok = iCartesianControl->movl(xd);
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
            CD_ERROR("size error\n");
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_MOVV)
    {
        if(in.size()>1)
        {
            std::vector<double> xdotd;
            for(size_t i=1; i<in.size();i++)
                xdotd.push_back(in.get(i).asDouble());
            bool ok = iCartesianControl->movv(xdotd);
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
            CD_ERROR("size error\n");
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_GCMP)
    {
        bool ok = iCartesianControl->gcmp();
        if(ok)
        {
            out.addVocab(VOCAB_OK);
        }
        else
        {
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_FORC)
    {
        if(in.size()>1)
        {
            std::vector<double> td;
            for(size_t i=1; i<in.size();i++)
                td.push_back(in.get(i).asDouble());
            bool ok = iCartesianControl->forc(td);
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
            CD_ERROR("size error\n");
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_STOP)
    {
        bool ok = iCartesianControl->stopControl();
        if(ok)
        {
            out.addVocab(VOCAB_OK);
        }
        else
        {
            out.addVocab(VOCAB_FAILED);
        }
    }
    else if( in.get(0).asVocab() == VOCAB_CC_TOOL)
    {
        if(in.size()>1)
        {
            std::vector<double> x;
            for(size_t i=1; i<in.size();i++)
                x.push_back(in.get(i).asDouble());
            bool ok = iCartesianControl->tool(x);
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

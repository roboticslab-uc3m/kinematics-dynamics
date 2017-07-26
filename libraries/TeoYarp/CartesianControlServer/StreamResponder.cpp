// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServer.hpp"

#include <ColorDebug.hpp>

// ------------------- StreamResponder Related ------------------------------------

void roboticslab::StreamResponder::onRead(yarp::os::Bottle& b)
{
    CD_DEBUG("Got: %s\n", b.toString().c_str());
    CD_DEBUG("Size: %d\n", b.size());
    switch (b.get(0).asVocab())
    {
    case VOCAB_CC_VMOS:
        {
            if (b.size() > 1)
            {
                std::vector<double> v;

                for (size_t i = 1; i < b.size(); i++)
                {
                    v.push_back(b.get(i).asDouble());
                }

                if (!iCartesianControl->vmos(v))
                {
                    CD_ERROR("vmos failed\n");
                }
            }
            else
            {
                CD_ERROR("size error\n");
            }
        }
        break;
    default:
        CD_ERROR("command not recognized\n");
        break;
    }
}

// -----------------------------------------------------------------------------

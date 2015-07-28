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
    if ((in.get(0).asString() == "help")||(in.get(0).asVocab() == VOCAB_HELP))  // help //
    {
        out.addString("Available commands: [help] [load]");
        return out.write(*returnToSender);
    }
    else if ((in.get(0).asString() == "load")||(in.get(0).asVocab() == VOCAB_LOAD))  // load //
    {
        if ( in.size() != 2 )
        {
            CD_ERROR("in.size() != 2\n");
            out.addVocab(VOCAB_FAILED);
            return out.write(*returnToSender);
        }
        std::string fileName = rf->findFileByName(in.get(1).asString());
        cartesianRateThread->ifs.open( fileName.c_str() );
        if( ! cartesianRateThread->ifs.is_open() )
        {
            CD_ERROR("Could not open read file: %s.\n",fileName.c_str());
            out.addVocab(VOCAB_FAILED);
            return out.write(*returnToSender);
        }
        CD_SUCCESS("Opened file: %s.\n",fileName.c_str());
        out.addVocab(VOCAB_OK);
        return out.write(*returnToSender);
    }
    else
    {
        fprintf(stderr,"[xRpcResponder] fail: Unknown command (use 'help' if needed).\n");
        out.addVocab(VOCAB_FAILED);
        return out.write(*returnToSender);
    }
}

/************************************************************************/

}  // namespace teo

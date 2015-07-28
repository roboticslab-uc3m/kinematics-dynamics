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

void TeoXRpcResponder::setPositionInterface(yarp::dev::IPositionControl* _ipos) {
    ipos = _ipos;
}

/************************************************************************/

}  // namespace teo

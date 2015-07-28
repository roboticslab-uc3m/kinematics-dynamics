// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoXCallbackPort.hpp"

namespace teo
{

/************************************************************************/

void TeoXCallbackPort::onRead(Bottle& b) {
    printf("[CartesianServerLib] xCallbackPort Got %s\n", b.toString().c_str());
}

/************************************************************************/

}  // namespace teo

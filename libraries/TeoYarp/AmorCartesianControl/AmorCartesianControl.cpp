// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <yarp/os/Time.h>

#include <ColorDebug.hpp>

bool roboticslab::AmorCartesianControl::waitForCompletion(int vocab)
{
    currentState = vocab;

    AMOR_RESULT res;
    amor_movement_status status;

    do
    {
        res = amor_get_movement_status(handle, &status);

        if (res == AMOR_FAILED)
        {
            CD_ERROR("%s\n", amor_error());
            break;
        }

        yarp::os::Time::delay(0.5);  // seconds
    }
    while (status != AMOR_MOVEMENT_STATUS_FINISHED);

    currentState = VOCAB_CC_NOT_CONTROLLING;

    return res == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

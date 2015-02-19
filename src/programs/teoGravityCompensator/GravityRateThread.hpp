// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RECORD_RATE_THREAD__
#define __RECORD_RATE_THREAD__

#include <vector>

#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "ColorDebug.hpp"

#define DEFAULT_MS 50  // [ms], overwritten by parent DEFAULT_PT_MODE_MS.

/**
 * @ingroup teoGravityCompensation
 *
 * The GravityRateThread class.
 *
 */
class GravityRateThread : public yarp::os::RateThread {

    public:
        // Set the Thread Rate in the class constructor
        GravityRateThread() : RateThread(DEFAULT_MS) {}  // In ms

        /** Loop function. This is the thread itself. */
        virtual void run();

        int rightArmNumMotors;
        yarp::dev::IEncoders *rightArmEnc;
        yarp::dev::ITorqueControl *rightArmTrq;

    protected:


};

#endif  // __RECORD_RATE_THREAD__


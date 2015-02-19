// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __GRAVITY_RATE_THREAD__
#define __GRAVITY_RATE_THREAD__

#include <vector>

#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "ColorDebug.hpp"
#include "ICartesianSolver.h"

#define DEFAULT_MS 50  // [ms], overwritten by parent DEFAULT_PT_MODE_MS.

namespace teo
{

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

        /** Initialization method. */
        virtual bool threadInit();

        /** Loop function. This is the thread itself. */
        virtual void run();

        /** Solver stuff */
        int solverNumLinksRA;
        teo::ICartesianSolver *solverRA;

        /** Robot stuff */
        int numMotorsRA;
        yarp::dev::IEncoders *iEncodersRA;
        std::vector< double > qRA;
        yarp::dev::ITorqueControl *iTorqueControlRA;


    protected:


};

}  // namespace teo

#endif  // __RECORD_RATE_THREAD__


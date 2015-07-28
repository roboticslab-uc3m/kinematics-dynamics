// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_RATE_THREAD__
#define __CARTESIAN_RATE_THREAD__

#include <vector>
#include <fstream>

#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "ColorDebug.hpp"
#include "ICartesianSolver.h"

#define DEFAULT_MS 50  // [ms], overwritten by parent DEFAULT_PT_MODE_MS.

namespace teo
{

/**
 * @ingroup teoCartesianServer
 *
 * @brief The actual \ref teoCartesianServer periodical thread.
 *
 */
class CartesianRateThread : public yarp::os::RateThread {

    public:
        // Set the Thread Rate in the class constructor
        CartesianRateThread() : RateThread(DEFAULT_MS) {}  // In ms

        /** Initialization method. */
        virtual bool threadInit();

        /** Loop function. This is the thread itself. */
        virtual void run();

        /** Solver stuff */
        int solverNumLinks;
        teo::ICartesianSolver *solver;

        /** Robot stuff */
        int numMotors;
        yarp::dev::IEncoders *iEncoders;
        yarp::dev::ITorqueControl *iTorqueControl;
        std::vector< double > q;
        std::vector< double > t;

        /** File stuff */
        std::ifstream ifs;

    protected:


};

}  // namespace teo

#endif  // __CARTESIAN_RATE_THREAD__


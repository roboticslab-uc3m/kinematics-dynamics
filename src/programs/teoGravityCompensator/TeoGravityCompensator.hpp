// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_GRAVITY_COMPENSATOR__
#define __TEO_GRAVITY_COMPENSATOR__

#include <yarp/os/RFModule.h>

#include <yarp/dev/all.h>

#include "ColorDebug.hpp"

#include "GravityRateThread.hpp"

#define DEFAULT_SOLVER "kdlsolver"
#define DEFAULT_KINEMATICS "leftArmKinematics.ini"
#define DEFAULT_REMOTE "/teo/leftArm"

namespace teo
{

/**
 * @ingroup TeoGravityCompensator
 *
 * @brief Uses an inverse dynamics solver to perform gravity compensation on a robot limb.
 * 
 */
class TeoGravityCompensator : public yarp::os::RFModule {

    public:

        TeoGravityCompensator() {}
        bool configure(yarp::os::ResourceFinder &rf);

    protected:

        GravityRateThread gravityRateThread;

        yarp::dev::PolyDriver solverDevice;

        yarp::dev::PolyDriver robotDevice;

        bool updateModule();
        bool interruptModule();
        // double getPeriod();


};

}  // namespace teo

#endif  // __TEO_GRAVITY_COMPENSATOR__


#ifndef __RATE_CONTROLLER_CONSOLE_HPP__
#define __RATE_CONTROLLER_CONSOLE_HPP__

#include <vector>
#include <functional>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IVelocityControl.h>

#include "ICartesianControl.h"

#define DEFAULT_ROBOT_LOCAL "/RateControllerClient"
#define DEFAULT_ROBOT_REMOTE "/asibot/asibotManipulator"

#define DEFAULT_CARTESIAN_LOCAL "/RateCartesianControlClient"
#define DEFAULT_CARTESIAN_REMOTE "/asibotSim/BasicCartesianControl"

namespace roboticslab
{

/**
 * @ingroup rateControllerConsole
 *
 * @brief TBD
 */
class RateControllerConsole : public yarp::os::RFModule
{
public:
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual double getPeriod();
    virtual bool close();

private:
    // used for array indexes and size checks
    enum joint { Q1 = 0, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, MAX_JOINTS };
    enum cart { X = 0, Y, Z, ROTX, ROTY, ROTZ, NUM_CART_COORDS };

    std::plus<double> increment_functor;
    std::minus<double> decrement_functor;

    template <typename func>
    void incrementOrDecrementJointVelocity(joint q, func op);

    template <typename func>
    void incrementOrDecrementCartesianVelocity(cart coord, func op);

    void printJointPositions();
    void printCartesianPositions();

    void issueStop();

    int axes;

    yarp::dev::PolyDriver controlboardDevice;
    yarp::dev::PolyDriver cartesianControlDevice;

    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IControlLimits2 * iControlLimits;
    yarp::dev::IVelocityControl * iVelocityControl;

    roboticslab::ICartesianControl * iCartesianControl;

    std::vector<double> maxVelocityLimits;
    std::vector<double> currentJointVels;
    std::vector<double> currentCartVels;

    static const double JOINT_VELOCITY_STEP;
    static const double CARTESIAN_LINEAR_VELOCITY_STEP;
    static const double CARTESIAN_ANGULAR_VELOCITY_STEP;
};

}  // namespace roboticslab

#endif  // __RATE_CONTROLLER_CONSOLE_HPP__

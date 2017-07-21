// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ASIBOT_SOLVER_HPP__
#define __ASIBOT_SOLVER_HPP__

#include <iostream> // only windows
#include <stdlib.h> // for exit()
#include <cmath>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include "ICartesianSolver.h"
#include "TrajGen.hpp"
#include "YarpTinyMath.hpp"
#include "AsibotConfiguration.hpp"

#define NUM_MOTORS 5
#define CARTPOS_PRECISION 0.01  /// Meter 0.0005
#define CARTORI_PRECISION 1.5  /// Degrees
#define GAIN 0  /// 75 good for unstabilized sim and common real. 25 ok with stable sim.

#define DEFAULT_A0 0.3
#define DEFAULT_A1 0.4
#define DEFAULT_A2 0.4
#define DEFAULT_A3 0.3
#define DEFAULT_CMC_MS 30  // ms
#define DEFAULT_DURATION 20  // For Trajectory
#define DEFAULT_MAXACC 0.2  // units/s^2
#define DEFAULT_MAXVEL 7.5  // units/s
#define DEFAULT_ROBOT_DEVICE "controlboardwrapper2"
#define DEFAULT_ROBOT_SUBDEVICE "ravebot"
#define DEFAULT_ROBOT_NAME "/ravebot"
#define DEFAULT_ROBOT_LOCAL "N/A"
#define DEFAULT_ROBOT_REMOTE "N/A"
#define DEFAULT_TOOL 0

namespace roboticslab
{

/**
 * @ingroup TeoYarp
 * @defgroup AsibotSolver
 *
 * @brief Contains roboticslab::AsibotSolver.
 */

/**
 * @ingroup AsibotSolver
 * @brief The AsibotSolver implements ICartesianSolver.
 */

class AsibotSolver : public yarp::dev::DeviceDriver, public ICartesianSolver
{
public:

    AsibotSolver() {}

    // -- ICartesianSolver declarations. Implementation in ICartesianSolverImpl.cpp--
    /** Get number of joints for which the solver has been configured. */
    virtual bool getNumJoints(int* numJoints);

    /** Append an additional link. */
    virtual bool appendLink(const std::vector<double> &x);

    /** Restore original kinematic chain. */
    virtual bool restoreOriginalChain();

    /** Perform forward kinematics. */
    virtual bool fwdKin(const std::vector<double> &q, std::vector<double> &x);

    /** Obtain error with respect to forward kinematics. */
    virtual bool fwdKinError(const std::vector<double> &xd, const std::vector<double> &q, std::vector<double> &x);

    /** Perform inverse kinematics. */
    virtual bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q);

    /** Perform differential inverse kinematics. */
    virtual bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot);

    /** Perform inverse dynamics. */
    virtual bool invDyn(const std::vector<double> &q, std::vector<double> &t);

    /** Perform inverse dynamics. */
    virtual bool invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t);

    /** Set joint limits. */
    virtual bool setLimits(const std::vector<double> &qMin, const std::vector<double> &qMax);

// ------- ICartesianControl declarations. Implementation in ICartesianImpl.cpp -------

    /**
    * Move the end-effector to a specified pose (position
    * and orientation) in cartesian space. [do not wait for reply]
    * @param xd a 3-d vector which contains the desired position 
    *           x,y,z
    * @param od a 2-d vector which contains the desired orientation
    * using axis-angle representation (xa, ya, za, theta). 
    * @param t set the trajectory duration time (seconds). If t< 
    *         (as by default) the current execution time is kept.
    * @return true/false on success/failure. 
    *  
    * @note Intended for streaming mode. 
    */
    virtual bool goToPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                          const double t=0);

    /**
    * Move the end-effector to a specified pose (position
    * and orientation) in cartesian space. [wait for reply]
    * @param xd a 3-d vector which contains the desired position 
    *          x,y,z (meters).
    * @param od a 2-d vector which contains the desired orientation
    * using axis-angle representation (xa, ya, za, theta). 
    * @param t set the trajectory duration time (seconds). If t<=0 
    *         (as by default) the current execution time is kept.
    * @return true/false on success/failure.
    */
    virtual bool goToPoseSync(const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                              const double t=0.0);

    /**
    * Set the reference velocities of the end-effector in the task 
    * space.
    * @param xdot the 3-d vector containing the x,y,z reference 
    *             velocities [m/s] of the end-effector.
    * @param odot the 2-d vector containing the orientation 
    *             reference velocity [rad/s] of the end-effector
    * @return true/false on success/failure.
    */
    virtual bool setTaskVelocities(const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot);

// -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    /**
    * Open the DeviceDriver. 
    * @param config is a list of parameters for the device.
    * Which parameters are effective for your device can vary.
    * See \ref dev_examples "device invocation examples".
    * If there is no example for your device,
    * you can run the "yarpdev" program with the verbose flag
    * set to probe what parameters the device is checking.
    * If that fails too,
    * you'll need to read the source code (please nag one of the 
    * yarp developers to add documentation for your device).
    * @return true/false upon success/failure
    */
    virtual bool open(yarp::os::Searchable& config);

    /**
    * Close the DeviceDriver.
    * @return true/false on success/failure.
    */
    virtual bool close();

// -------- RateThread declarations. Implementation in RateThreadImpl.cpp --------

  /**
   * Initialization method. The thread executes this function
   * when it starts and before "run". This is a good place to
   * perform initialization tasks that need to be done by the
   * thread itself (device drivers initialization, memory
   * allocation etc). If the function returns false the thread
   * quits and never calls "run". The return value of threadInit()
   * is notified to the class and passed as a parameter
   * to afterStart(). Note that afterStart() is called by the
   * same thread that is executing the "start" method.
   */
  bool threadInit();

  /**
   * Loop function. This is the thread itself.
   */
  void run();

protected:

  /**
   * Check reachability of specified joint.
   * @param joint zero-based joint number
   * @param qInDeg joint angle in [deg]
   * @return true if joint within limits, false otherwise
   */
  bool checkJointInLimits(int joint, double qInDeg)
  {
      return qInDeg >= qMin[joint] && qInDeg <= qMax[joint];
  }

private:

    bool withOri;
    int tool;

    Traj *trajOz, *trajPrP, *trajPhP, *trajOyP, *trajOzPP;
    yarp::sig::Vector targetX, targetO;
    yarp::sig::Vector realRad;  // current radians

    double A0, A1, A2, A3;  // link lengths
    double startTime;

    std::vector<double> qMin, qMax;

    AsibotConfiguration * conf;
};

}  // namespace roboticslab

#endif  // __ASIBOT_SOLVER_HPP__

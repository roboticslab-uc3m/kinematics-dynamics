// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ASIBOT_SOLVER_HPP__
#define __ASIBOT_SOLVER_HPP__

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

#include <iostream> // only windows
#include <stdlib.h> // for exit()


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

using namespace teo;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

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

class AsibotSolver : public DeviceDriver, public teo::ICartesianSolver
{
public:

    AsibotSolver() {}  // In ms

    // -- ICartesianSolver declarations. Implementation in ICartesianSolverImpl.cpp--
    /** Get number of links for which the solver has been configured. */
    virtual bool getNumLinks(int* numLinks);

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

// -- Helper Funcion declarations. Implementation in HelperFuncs.cpp--

    /**
    * Perform forward kinematics.
    */
    bool fwdKin(const double inDeg[NUM_MOTORS], yarp::sig::Vector &x, yarp::sig::Vector &o);

// ------- ICartesianControl declarations. Implementation in ICartesianImpl.cpp -------

    /**
    * Set the controller in tracking or non-tracking mode. [wait for
    * reply] 
    * @param f true for tracking mode, false otherwise. 
    * @return true/false on success/failure. 
    *  
    * @note In tracking mode when the controller reaches the target,
    *       it keeps on running in order to maintain the limb in the
    *       desired pose in the presence of external disturbances.
    *       In non-tracking mode the controller releases the limb as
    *       soon as the desired pose is reached.
    */
    virtual bool setTrackingMode(const bool f);

    /**
    * Get the current controller mode. [wait for reply]
    * @param f here is returned true if controller is in tracking 
    *         mode, false otherwise.
    * @return true/false on success/failure. 
    */
    virtual bool getTrackingMode(bool *f);

    /**
    * Ask the controller to close the loop with the low-level joints
    * set-points in place of the actual encoders feedback. [wait for
    * reply] 
    * @param f true for reference mode, false otherwise. 
    * @return true/false on success/failure. 
    *  
    * @note When the reference mode is enabled the controller makes 
    *       use of the low-level joints set-points that result from
    *       the integration of the velocity commands in place of the
    *       actual encoders feedbacks. This modality is particularly
    *       useful in a scenario where the velocity commands are
    *       executed by the control boards with resort to torque
    *       actuation.
    */
    virtual bool setReferenceMode(const bool f);

    /**
    * Get the current controller reference mode. [wait for reply]
    * @param f here is returned true if controller makes use of the 
    *         low-level joints set-points, false if it employs
    *         actual encoders feedback.
    * @return true/false on success/failure. 
    */
    virtual bool getReferenceMode(bool *f);

    /*!
     * Ask the controller to weigh more either the position or the
     * orientation while reaching in full pose. [wait for reply]
     * \param p can be "position" or "orientation".
     * \return true/false on success/failure.
     */
    virtual bool setPosePriority(const yarp::os::ConstString &p) { return true; }

    /*!
     * Get the current pose priority. [wait for reply]
     * \param p here is returned either as "position" or "orientation".
     * \return true/false on success/failure.
     */
    virtual bool getPosePriority(yarp::os::ConstString &p) { return true; }

    /**
    * Get the current pose of the end-effector. [do not wait for 
    * reply] 
    * @param x a 3-d vector which is filled with the actual 
    *         position x,y,z (meters).
    * @param od a 2-d vector which is filled with the actual 
    * orientation using euler representation xa, ya, za, theta 
    * (meters and radians). 
    * @param stamp the stamp of the encoders employed to compute the
    *              pose.
    * @return true/false on success/failure.
    */
    virtual bool getPose(yarp::sig::Vector &x, yarp::sig::Vector &o,
                         yarp::os::Stamp *stamp=NULL);

    /**
    * Get the current pose of the specified link belonging to the 
    * kinematic chain. [wait for reply] 
    * @param axis joint index (regardless if it is actuated or 
    *            not). 
    * @param x a 3-d vector which is filled with the actual position
    *         x,y,z (meters) of the given link reference frame.
    * @param od a 2-d vector which is filled with the actual 
    * orientation of the given link reference frame using axis-angle
    * representation xa, ya, za, theta (meters and radians).
    * @param stamp the stamp of the encoders employed to compute the
    *              pose.
    * @return true/false on success/failure.
    */
    virtual bool getPose(const int axis, yarp::sig::Vector &x, yarp::sig::Vector &o,
                         yarp::os::Stamp *stamp=NULL);

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
    * Move the end-effector to a specified position in cartesian 
    * space, ignore the orientation. [do not wait for reply] 
    * @param xd a 3-d vector which contains the desired position 
    *          x,y,z (meters).
    * @param t set the trajectory duration time (seconds). If t<=0 
    *         (as by default) the current execution time is kept. 
    * @return true/false on success/failure. 
    *  
    * @note Intended for streaming mode. 
    */
    virtual bool goToPosition(const yarp::sig::Vector &xd, const double t=0.0);

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
    * Move the end-effector to a specified position in cartesian 
    * space, ignore the orientation. [wait for reply] 
    * @param xd a 3-d vector which contains the desired position 
    *          x,y,z (meters).
    * @param t set the trajectory duration time (seconds). If t<=0 
    *         (as by default) the current execution time is kept. 
    * @return true/false on success/failure.
    */
    virtual bool goToPositionSync(const yarp::sig::Vector &xd, const double t=0.0);

    /**
    * Get the actual desired pose and joints configuration as result
    * of kinematic inversion. [wait for reply] 
    * @param xdhat a 3-d vector which is filled with the actual 
    *          desired position x,y,z (meters); it may differ from
    *          the commanded xd.
    * @param odhat a 2-d vector which is filled with the actual 
    *          desired orientation using axis-angle representation
    *          xa, ya, za, theta (meters and radians); it may differ
    *          from the commanded od. 
    * @param qdhat the joints configuration through which the
    *             couple (xdhat,odhat) is achieved (degrees).
    * @return true/false on success/failure.
    */
    virtual bool getDesired(yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat);

    /**
    * Ask for inverting a given pose without actually moving there.
    * [wait for reply] 
    * @param xd a 3-d vector which contains the desired 
    *          position x,y,z (meters).
    * @param od a 2-d vector which contains the desired 
    *          orientation using axis-angle representation xa, ya,
    *          za, theta (meters and radians).
    * @param xdhat a 3-d vector which is filled with the final 
    *          position x,y,z (meters); it may differ from the
    *          commanded xd.
    * @param odhat a 2-d vector which is filled with the final 
    *          orientation using axis-angle representation xa, ya,
    *          za, theta (meters and radians); it may differ from
    *          the commanded od.
    * @param qdhat the joints configuration through which the
    *             couple (xdhat,odhat) is achieved (degrees).
    * @return true/false on success/failure.
    */
    virtual bool askForPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                            yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat);

    /**
    * Ask for inverting a given pose without actually moving there.
    * [wait for reply] 
    * @param q0 a vector of length DOF which contains the starting
    *           joints configuration (degrees), made compatible with
    *           the chain.
    * @param xd a 3-d vector which contains the desired 
    *          position x,y,z (meters).
    * @param od a 2-d vector which contains the desired 
    *          orientation using axis-angle representation xa, ya,
    *          za, theta (meters and radians).
    * @param xdhat a 3-d vector which is filled with the final 
    *          position x,y,z (meters); it may differ from the
    *          commanded xd.
    * @param odhat a 2-d vector which is filled with the final 
    *          orientation using axis-angle representation xa, ya,
    *          za, theta (meters and radians); it may differ from
    *          the commanded od.
    * @param qdhat the joints configuration through which the
    *             couple (xdhat,odhat) is achieved (degrees).
    * @return true/false on success/failure.
    */
    virtual bool askForPose(const yarp::sig::Vector &q0,
                            const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                            yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                            yarp::sig::Vector &qdhat);

    /**
    * Ask for inverting a given position without actually moving 
    * there. [wait for reply] 
    * @param xd a 3-d vector which contains the desired 
    *          position x,y,z (meters).
    * @param xdhat a 3-d vector which is filled with the final 
    *          position x,y,z (meters); it may differ from the
    *          commanded xd.
    * @param odhat a 2-d vector which is filled with the final 
    *          orientation using axis-angle representation xa, ya,
    *          za, theta (meters and radians); it may differ from
    *          the commanded od.
    * @param qdhat the joints configuration through which the
    *             couple (xdhat,odhat) is achieved (degrees).
    * @return true/false on success/failure.
    */
    virtual bool askForPosition(const yarp::sig::Vector &xd,
                                yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                                yarp::sig::Vector &qdhat);

    /**
    * Ask for inverting a given position without actually moving 
    * there. [wait for reply] 
    * @param q0 a vector of length DOF which contains the starting 
    *           joints configuration (degrees), made compatible with
    *           the chain.
    * @param xd a 3-d vector which contains the desired 
    *          position x,y,z (meters).
    * @param xdhat a 3-d vector which is filled with the final 
    *          position x,y,z (meters); it may differ from the
    *          commanded xd.
    * @param odhat a 2-d vector which is filled with the final 
    *          orientation using axis-angle representation xa, ya,
    *          za, theta (meters and radians); it may differ from
    *          the commanded od.
    * @param qdhat the joints configuration through which the
    *             couple (xdhat,odhat) is achieved (degrees).
    * @return true/false on success/failure.
    */
    virtual bool askForPosition(const yarp::sig::Vector &q0,
                                const yarp::sig::Vector &xd,
                                yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                                yarp::sig::Vector &qdhat);

    /**
    * Get the current DOF configuration of the limb. [wait for
    * reply] 
    * @param curDof a vector which is filled with the actual DOF 
    *           configuration.
    * @return true/false on success/failure. 
    *  
    * @note The vector lenght is equal to the number of limb's 
    *       joints; each vector's position is filled with 1 if the
    *       associated joint is controlled (i.e. it is an actuated
    *       DOF), 0 otherwise.
    */
    virtual bool getDOF(yarp::sig::Vector &curDof);

    /**
    * Set a new DOF configuration for the limb. [wait for reply]
    * @param newDof a vector which contains the new DOF 
    *            configuration.
    * @param curDof a vector where the DOF configuration is 
    *              returned as it has been processed after the
    *              request (it may differ from newDof due to the
    *              presence of some internal limb's constraints).
    * @return true/false on success/failure. 
    *  
    * @note Eeach vector's position shall contain 1 if the 
    *       associated joint can be actuated, 0 otherwise. The
    *       special value 2 indicates that the joint status won't be
    *       modified (useful as a placeholder).
    */
    virtual bool setDOF(const yarp::sig::Vector &newDof, yarp::sig::Vector &curDof);

    /**
    * Get the current joints rest position. [wait for reply]
    * @param curRestPos a vector which is filled with the current 
    *                  joints rest position components in degrees.
    * @return true/false on success/failure. 
    *  
    * @note While solving the inverse kinematic, the user may 
    *       specify a secondary task that minimizes against a joints
    *       rest position; further, each rest component may be
    *       weighted differently providing the weights vector.
    */
    virtual bool getRestPos(yarp::sig::Vector &curRestPos);

    /**
    * Set a new joints rest position. [wait for reply] 
    * @param newRestPos a vector which contains the new joints rest
    *                  position components in degrees.
    * @param curRestPos a vector which is filled with the current 
    *           joints rest position components in degrees as result
    *           from thresholding with joints bounds.
    * @return true/false on success/failure. 
    *  
    * @note While solving the inverse kinematic, the user may 
    *       specify a secondary task that minimizes against a joints
    *       rest position; further, each rest component may be
    *       weighted differently providing the weights vector.
    */
    virtual bool setRestPos(const yarp::sig::Vector &newRestPos, yarp::sig::Vector &curRestPos);

    /**
    * Get the current joints rest weights. [wait for reply]
    * @param curRestWeights a vector which is filled with the 
    *                  current joints rest weights.
    * @return true/false on success/failure. 
    *  
    * @note While solving the inverse kinematic, the user may 
    *       specify a secondary task that minimizes against a joints
    *       rest position; further, each rest component may be
    *       weighted differently providing the weights vector.
    */
    virtual bool getRestWeights(yarp::sig::Vector &curRestWeights);

    /**
    * Set a new joints rest position. [wait for reply] 
    * @param newRestWeights a vector which contains the new joints 
    *                  rest weights.
    * @param curRestWeights a vector which is filled with the 
    *           current joints rest weights as result from
    *           saturation (w>=0.0).
    * @return true/false on success/failure. 
    *  
    * @note While solving the inverse kinematic, the user may 
    *       specify a secondary task that minimizes against a joints
    *       rest position; further, each rest component may be
    *       weighted differently providing the weights vector.
    */
    virtual bool setRestWeights(const yarp::sig::Vector &newRestWeights, yarp::sig::Vector &curRestWeights);

    /**
    * Get the current range for the axis. [wait for reply]
    * @param axis joint index (regardless if it is actuated or 
    *            not).
    * @param min where the minimum value is returned [deg].
    * @param max where the maximum value is returned [deg].
    * @return true/false on success/failure. 
    */
    virtual bool getLimits(const int axis, double *min, double *max);

    /**
    * Set new range for the axis. Allowed range shall be a valid 
    * subset of the real control limits. [wait for reply]
    * @param axis joint index (regardless it it is actuated or 
    *            not).
    * @param min the new minimum value [deg]. 
    * @param max the new maximum value [deg]. 
    * @return true/false on success/failure. 
    */
    virtual bool setLimits(const int axis, const double min, const double max);

    /**
    * Get the current trajectory duration. [wait for reply]
    * @param t the memory location where the time is returned 
    *         (seconds).
    * @return true/false on success/failure. 
    */
    virtual bool getTrajTime(double *t);

    /**
    * Set the duration of the trajectory. [wait for reply]
    * @param t time (seconds).
    * @return true/false on success/failure. 
    */
    virtual bool setTrajTime(const double t);

    /**
    * Return tolerance for in-target check. [wait for reply]
    * @param tol the memory location where tolerance is returned. 
    * @return true/false on success/failure. 
    *  
    * @note The trajectory is supposed to be completed as soon as 
    *       norm(xd-end_effector)<tol.
    */
    virtual bool getInTargetTol(double *tol);

    /**
    * Set tolerance for in-target check. [wait for reply]
    * @param tol tolerance. 
    * @return true/false on success/failure. 
    *  
    * @note The trajectory is supposed to be completed as soon as 
    *       norm(xd-end_effector)<tol.
    */
    virtual bool setInTargetTol(const double tol);

    /**
    * Return joints velocities. [wait for reply] 
    * @param qdot the vector containing the joints velocities 
    *             [deg/s] sent to the robot by the controller.
    * @return true/false on success/failure.
    */
    virtual bool getJointsVelocities(yarp::sig::Vector &qdot);

    /**
    * Return velocities of the end-effector in the task space. [wait
    * for reply] 
    * @param xdot the 3-d vector containing the derivative of x,y,z 
    *             position [m/s] of the end-effector while moving in
    *             the task space as result of the commanded joints
    *             velocities.
    * @param odot the 2-d vector containing the derivative of 
    *             end-effector orientation [rad/s] while moving in
    *             the task space as result of the commanded joints
    *             velocities.
    * @return true/false on success/failure.
    */
    virtual bool getTaskVelocities(yarp::sig::Vector &xdot, yarp::sig::Vector &odot);

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

    /**
    * Attach a tip frame to the end-effector. 
    * @param x a 3-d vector describing the position of the tip frame
    *          wrt the end-effector (meters).
    * @param o a 4-d vector describing the orientation of the tip 
    *          frame wrt the end-effector (axis-angle notation).
    * @return true/false if successful/failed. 
    *  
    * @note By attaching a tip to the end-effector, the specified 
    *       tip will be the new end-effector for the controller.
    */
    virtual bool attachTipFrame(const yarp::sig::Vector &x, const yarp::sig::Vector &o);

    /**
    * Retrieve the tip frame currently attached to the end-effector.
    * @param x a 3-d vector containing the position of the tip frame
    *          wrt the end-effector (meters).
    * @param o a 4-d vector containing the orientation of the tip 
    *          frame wrt the end-effector (axis-angle notation).
    * @return true/false if successful/failed.
    */
    virtual bool getTipFrame(yarp::sig::Vector &x, yarp::sig::Vector &o);

    /**
    * Remove the tip frame currently attached to the end-effector.  
    * @return true/false if successful/failed. 
    *  
    * @note The actual end-effector is again under control.
    */
    virtual bool removeTipFrame();

    /** Check once if the current trajectory is terminated. [wait for
    *   reply]
    * @param f where the result is returned.
    * @return true/false on success/failure.
    */
    virtual bool checkMotionDone(bool *f);

    /** Wait until the current trajectory is terminated. [wait for
    *   reply]
    * @param period specify the check time period (seconds). 
    * @param timeout specify the check expiration time (seconds). If
    *         timeout<=0 (as by default) the check will be performed
    *         without time limitation.
    * @return true for success, false for failure and timeout 
    *         expired.
    */
    virtual bool waitMotionDone(const double period=0.1, const double timeout=0.0);

    /** Ask for an immediate stop motion. [wait for reply]
    * @return true/false on success/failure. 
    *  
    * @note The control is completely released, i.e. a direct switch
    *       to non-tracking mode is executed.     
    */
    virtual bool stopControl();

    /** Store the controller context. [wait for reply]
    * @param id specify where to store the returned context id. 
    * @return true/false on success/failure. 
    *  
    * @note The context comprises the values of internal controller 
    *       variables, such as the tracking mode, the active dofs,
    *       the trajectory time and so on.
    */
    virtual bool storeContext(int *id);

    /** Restore the controller context previously stored. [wait for
    *   reply]
    * @param id specify the context id to be restored
    * @return true/false on success/failure. 
    *  
    * @note The context comprises the values of internal controller
    *       variables, such as the tracking mode, the active dofs,
    *       the trajectory time and so on.
    */
    virtual bool restoreContext(const int id);

    /**
    * Returns useful info on the operating state of the controller. 
    * [wait for reply] 
    * @param info is a property-like bottle containing the info.
    * @return true/false on success/failure. 
    */
    virtual bool getInfo(yarp::os::Bottle &info);

    /**
    * Register an event. 
    * @param event the event to be registered.
    * @return true/false on success/failure. 
    *  
    * @note the special type "*" can be used to attach a callback to
    *       all the available events.
    */
    virtual bool registerEvent(yarp::dev::CartesianEvent &event);

    /**
    * Unregister an event.
    * @param event the event to be unregistered.
    * @return true/false on success/failure. 
    */
    virtual bool unregisterEvent(yarp::dev::CartesianEvent &event);

    /**
    * Tweak low-level controller's parameters. [wait for reply]
    * @param options is a property-like bottle containing new values 
    *                for the low-level controller's configuration.
    * @return true/false on success/failure. 
    *  
    * @note This method is intended for accessing low-level 
    *       controller's configuration.
    */
    virtual bool tweakSet(const yarp::os::Bottle &options);

    /**
    * Return low-level controller's parameters. [wait for reply]
    * @param options is a property-like bottle containing the 
    *                current values of the low-level controller's
    *                configuration.
    * @return true/false on success/failure. 
    *  
    * @note This method is intended for accessing low-level 
    *       controller's configuration.
    */
    virtual bool tweakGet(yarp::os::Bottle &options);

    /** Delete a specified controller context. [wait for reply]
    * @param id specify the context id to be removed.
    * @return true/false on success/failure. 
    */
    virtual bool deleteContext(const int id);

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
    virtual bool open(Searchable& config);

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

private:
    Property options;
    PolyDriver robotDevice;
    IEncoders *enc;
    IVelocityControl *vel;
    IControlMode *mode;

    int cmc_status;
    bool withOri;
    int tool;

    Traj *trajOz, *trajPrP, *trajPhP, *trajOyP, *trajOzPP;
    yarp::sig::Vector targetX,targetO;
    yarp::sig::Vector realRad;  // current radians

    double A0, A1, A2, A3;  // link lengths
    double startTime;
    double duration, maxVel, maxAcc;
    
    double cmcMs;
    bool isQuiet;
};

}  // namespace roboticslab

#endif  // __ASIBOT_SOLVER_HPP__

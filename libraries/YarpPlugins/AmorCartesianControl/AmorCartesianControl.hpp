// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __AMOR_CARTESIAN_CONTROL_HPP__
#define __AMOR_CARTESIAN_CONTROL_HPP__

#include <yarp/os/Searchable.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>

#include <amor.h>

#include "ICartesianControl.h"
#include "ICartesianSolver.h"

#define DEFAULT_CAN_LIBRARY "libeddriver.so"
#define DEFAULT_CAN_PORT 0

#define DEFAULT_GAIN 0.05
#define DEFAULT_QDOT_LIMIT 10.0
#define DEFAULT_WAIT_PERIOD_MS 30
#define DEFAULT_REFERENCE_FRAME "base"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup AmorCartesianControl
 * @brief Contains roboticslab::AmorCartesianControl.
 */

/**
 * @ingroup AmorCartesianControl
 * @brief The AmorCartesianControl class implements ICartesianControl.
 *
 * Uses the roll-pitch-yaw (RPY) angle representation.
 */

class AmorCartesianControl : public yarp::dev::DeviceDriver, public ICartesianControl
{
public:

    AmorCartesianControl() : handle(AMOR_INVALID_HANDLE),
                             ownsHandle(false),
                             iCartesianSolver(NULL),
                             currentState(VOCAB_CC_NOT_CONTROLLING),
                             gain(DEFAULT_GAIN),
                             maxJointVelocity(DEFAULT_QDOT_LIMIT),
                             waitPeriodMs(DEFAULT_WAIT_PERIOD_MS),
                             referenceFrame(ICartesianSolver::BASE_FRAME)
    {}

    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --

    virtual bool stat(std::vector<double> &x, int * state = 0, double * timestamp = 0);

    virtual bool inv(const std::vector<double> &xd, std::vector<double> &q);

    virtual bool movj(const std::vector<double> &xd);

    virtual bool relj(const std::vector<double> &xd);

    virtual bool movl(const std::vector<double> &xd);

    virtual bool movv(const std::vector<double> &xdotd);

    virtual bool gcmp();

    virtual bool forc(const std::vector<double> &td);

    virtual bool stopControl();

    virtual bool wait(double timeout);

    virtual bool tool(const std::vector<double> &x);

    virtual void twist(const std::vector<double> &xdot);

    virtual void pose(const std::vector<double> &x, double interval);

    virtual void movi(const std::vector<double> &x);

    virtual bool setParameter(int vocab, double value);

    virtual bool getParameter(int vocab, double * value);

    virtual bool setParameters(const std::map<int, double> & params);

    virtual bool getParameters(std::map<int, double> & params);

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------

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

private:

    bool checkJointVelocities(const std::vector<double> &qdot);

    AMOR_HANDLE handle;
    bool ownsHandle;

    yarp::dev::PolyDriver cartesianDevice;
    roboticslab::ICartesianSolver *iCartesianSolver;

    int currentState;

    double gain;
    double maxJointVelocity;
    int waitPeriodMs;

    ICartesianSolver::reference_frame referenceFrame;
};

}  // namespace roboticslab

#endif  // __AMOR_CARTESIAN_CONTROL_HPP__

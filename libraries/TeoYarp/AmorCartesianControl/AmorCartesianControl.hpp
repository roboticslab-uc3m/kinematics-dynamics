// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __AMOR_CARTESIAN_CONTROL_HPP__
#define __AMOR_CARTESIAN_CONTROL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <amor.h>

#include "ICartesianControl.h"
#include "ColorDebug.hpp"

#define DEFAULT_CAN_LIBRARY "libeddriver.so"
#define DEFAULT_CAN_PORT 0

namespace roboticslab
{

/**
 * @ingroup TeoYarp
 * @defgroup AmorCartesianControl
 * @brief Contains teo::AmorCartesianControl.
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
                             ownsHandle(false)
    {}

    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --

    /** Inform on control state, and get robot position and perform forward kinematics. */
    virtual bool stat(int &state, std::vector<double> &x);

    /** Perform inverse kinematics (using robot position as initial guess) but do not move. */
    virtual bool inv(const std::vector<double> &xd, std::vector<double> &q);

    /** movj */
    virtual bool movj(const std::vector<double> &xd);

    /** relj */
    virtual bool relj(const std::vector<double> &xd);

    /** movl */
    virtual bool movl(const std::vector<double> &xd);

    /** movv */
    virtual bool movv(const std::vector<double> &xdotd);

    /** gcmp */
    virtual bool gcmp();

    /** forc */
    virtual bool forc(const std::vector<double> &td);

    /** stop */
    virtual bool stopControl();

    /** act */
    virtual bool act(int commandCode);

    /** fwd */
    virtual bool fwd(const std::vector<double> &rot);

    /** bkwd*/
    virtual bool bkwd(const std::vector<double> &rot);

    /** rot */
    virtual bool rot(const std::vector<double> &rot);

    /** vmos */
    virtual bool vmos(const std::vector<double> &xdot);

    /** pose */
    virtual bool pose(const std::vector<double> &x);

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

protected:

    static double toDeg(double rad)
    {
        return rad * 180 / M_PI;
    }

    static double toRad(double deg)
    {
        return deg * M_PI / 180;
    }

private:

    AMOR_HANDLE handle;
    bool ownsHandle;
};

}  // namespace roboticslab

#endif  // __AMOR_CARTESIAN_CONTROL_HPP__

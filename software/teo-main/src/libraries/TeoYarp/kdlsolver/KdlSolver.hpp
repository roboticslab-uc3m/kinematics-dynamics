// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_SOLVER_HPP__
#define __KDL_SOLVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <kdl/segment.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>

#include <kdl/chainiksolverpos_lma.hpp>

#include <iostream> // only windows
#include <stdlib.h> // for exit()

#include "ColorDebug.hpp"
#include "../ICartesianSolver.h"

#define GAIN 0.25  /// 75 good for unstabilized sim and common real. 25 ok with stable sim.

#define DEFAULT_ANGLE_REPR "RPY"  // string
#define DEFAULT_NUM_LINKS 1  // int

#define DEFAULT_EPSILON 0.005     // Precision tolerance
#define DEFAULT_DURATION 20     // For Trajectory
#define DEFAULT_MAXVEL 7.5      // unit/s
#define DEFAULT_MAXACC 0.2      // unit/s^2
#define DEFAULT_ROBOT_DEVICE "remote_controlboard"
#define DEFAULT_ROBOT_SUBDEVICE "N/A"
#define DEFAULT_ROBOT_NAME "N/A"
#define DEFAULT_ROBOT_LOCAL "/KdlSolver/rightArm"
#define DEFAULT_ROBOT_REMOTE "/teoSim/rightArm"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;
using namespace KDL;

namespace teo
{

/**
 * @ingroup teo_yarp
 * \defgroup KdlSolver
 *
 * The \ref KdlSolver library is composed by a single class, KdlSolver.
 *
 * @section kdlsolver_install Installation
 *
 * The plugin is compiled when ENABLE_TeoYarp_cartesianbot is activated (not default). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 */

/**
 * @ingroup KdlSolver
 * @brief The KdlSolver class exposes a YARP_dev cartesian interface (implements
 * <a href="http://eris.liralab.it/yarpdoc/classyarp_1_1dev_1_1ICartesianControl.html">ICartesianControl</a>).
 */

class KdlSolver : public DeviceDriver, public ICartesianSolver {

    public:

        // Set the Thread Rate in the class constructor
        KdlSolver() {}  // In ms

        // -- ICartesianSolver declarations. Implementation in ICartesianSolverImpl.cpp--
        /**
        * Perform forward kinematics.
        */
        bool fwdKin(const yarp::sig::Vector &inUnits, yarp::sig::Vector &x, yarp::sig::Vector &o);

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

    private:

        Property options;
        PolyDriver robotDevice;
        IEncoders *enc;
        IPositionControl *pos;
        IVelocityControl *vel;
        IControlLimits *lim;

        bool withOri;

        Trajectory_Segment* currentTrajectory;
        RotationalInterpolation_SingleAxis* _orient;
        double _eqradius;
        bool _aggregate;

        yarp::sig::Vector isPrismatic;
        KDL::Frame targetF;
        yarp::sig::Vector targetO;

        Chain theChain;

        double startTime;

        int numLinks;
        std::string angleRepr;
        double epsilon, duration, maxVel, maxAcc, cmcMs;

    protected:
        /**
        * Simple function to pass from radians to degrees.
        * @param inRad angle value in radians.
        * @return angle value in degrees.
        */
        double toDeg(const double inRad) {
            return (inRad * 180.0 / M_PI);  // return (inRad * 180.0 / 3.14159265);
        }

        /**
        * Simple function to pass from degrees to radians.
        * @param inDeg angle value in degrees.
        * @return angle value in radians.
        */
        double toRad(const double inDeg) {
            return (inDeg * M_PI / 180.0);  // return (inDeg * 3.14159265 / 180.0);
        }

};

}  // namespace teo

#endif  // __KDL_SOLVER_HPP__


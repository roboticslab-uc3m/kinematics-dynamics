// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_SOLVER_HPP__
#define __KDL_SOLVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>

#include <kdl/segment.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <iostream> // only windows

#include "ColorDebug.hpp"
#include "ICartesianSolver.h"
#include "KdlVectorConverter.hpp"

#define DEFAULT_ANGLE_REPR "axisAngle"  // string
#define DEFAULT_NUM_LINKS 1  // int

#define DEFAULT_EPSILON 0.005     // Precision tolerance
#define DEFAULT_DURATION 20     // For Trajectory
#define DEFAULT_MAXVEL 7.5      // unit/s
#define DEFAULT_MAXACC 0.2      // unit/s^2

//#define _USE_LMA_

namespace teo
{

/**
 * @ingroup TeoYarp
 * \defgroup KdlSolver
 *
 * @brief Contains teo::KdlSolver.
 */

/**
 * @ingroup KdlSolver
 * @brief The KdlSolver class implements ICartesianSolver.
 */

class KdlSolver : public yarp::dev::DeviceDriver, public ICartesianSolver, public KdlVectorConverter {

    public:

        KdlSolver() : KdlVectorConverter(DEFAULT_ANGLE_REPR) {}

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

    protected:

        /** The chain. **/
        KDL::Chain chain;

        /** Number of links of the chain. **/
        int numLinks;

        /** Define used gravity for the chain, important to think of DH. **/
        KDL::Vector gravity;

        /** Minimum joint limits. **/
        KDL::JntArray qMin;

        /** Maximum joint limits. **/
        KDL::JntArray qMax;

        bool getMatrixFromProperties(yarp::os::Searchable &options, std::string &tag, yarp::sig::Matrix &H);

};

}  // namespace teo

#endif  // __KDL_SOLVER_HPP__


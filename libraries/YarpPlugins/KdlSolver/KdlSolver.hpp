// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_SOLVER_HPP__
#define __KDL_SOLVER_HPP__

#include <mutex>

#include <yarp/dev/DeviceDriver.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainidsolver.hpp>

#include "ICartesianSolver.h"
#include "KdlSolver_ParamsParser.h"
#include "LogComponent.hpp"

/**
 * @ingroup YarpPlugins
 * @defgroup KdlSolver
 *
 * @brief Contains KdlSolver.
 */

/**
 * @ingroup KdlSolver
 * @brief The KdlSolver class implements ICartesianSolver.
 */
class KdlSolver : public yarp::dev::DeviceDriver,
                  public roboticslab::ICartesianSolver,
                  public KdlSolver_ParamsParser
{
public:
    // -- ICartesianSolver declarations. Implementation in ICartesianSolverImpl.cpp --

    // Get number of joints for which the solver has been configured.
    yarp::dev::ReturnValue getNumJoints(std::size_t & numJoints) override;

    // Get number of TCPs for which the solver has been configured.
    yarp::dev::ReturnValue getNumTcps(std::size_t & numTcps) override;

    // Append an additional link.
    yarp::dev::ReturnValue appendLink(const std::vector<double> & x) override;

    // Restore original kinematic chain.
    yarp::dev::ReturnValue restoreOriginalChain() override;

    // Change reference frame.
    yarp::dev::ReturnValue changeOrigin(const std::vector<double> & x_old_obj, const std::vector<double> & x_new_old, std::vector<double> & x_new_obj) override;

    // Perform forward kinematics.
    yarp::dev::ReturnValue forwardKinematics(const std::vector<double> & q, std::vector<double> & x) override;

    // Obtain difference between supplied pose inputs.
    yarp::dev::ReturnValue poseDiff(const std::vector<double> & xLhs, const std::vector<double> & xRhs, std::vector<double> & xOut) override;

    // Perform inverse kinematics.
    yarp::dev::ReturnValue inverseKinematics(const std::vector<double> & xd, const std::vector<double> & qGuess, std::vector<double> & q, Frame frame) override;

    // Perform differential inverse kinematics.
    yarp::dev::ReturnValue diffInverseKinematics(const std::vector<double> & q, const std::vector<double> & xdot, std::vector<double> & qdot, Frame frame) override;

    // Perform inverse dynamics.
    yarp::dev::ReturnValue inverseDynamics(const std::vector<double> & q, std::vector<double> & t) override;

    // Perform inverse dynamics.
    yarp::dev::ReturnValue inverseDynamics(const std::vector<double> & q, const std::vector<double> & qdot, const std::vector<double> & qdotdot,
                                           const std::vector<double> & ftip, std::vector<double> & t, Frame frame) override;

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    inline const yarp::os::LogComponent & logc() const
    { return !m_quiet ? KDLS() : KDLS_QUIET(); }

    mutable std::mutex mtx;

    KDL::Chain chain;
    KDL::Chain originalChain;

    KDL::ChainFkSolverPos * fkSolverPos {nullptr};
    KDL::ChainIkSolverPos * ikSolverPos {nullptr};
    KDL::ChainIkSolverVel * ikSolverVel {nullptr};
    KDL::ChainIdSolver * idSolver {nullptr};
};

#endif // __KDL_SOLVER_HPP__

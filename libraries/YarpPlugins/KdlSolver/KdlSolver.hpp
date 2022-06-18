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
#include "LogComponent.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup KdlSolver
 *
 * @brief Contains roboticslab::KdlSolver.
 */

/**
 * @ingroup KdlSolver
 * @brief The KdlSolver class implements ICartesianSolver.
 */
class KdlSolver : public yarp::dev::DeviceDriver,
                  public ICartesianSolver
{
public:
    // -- ICartesianSolver declarations. Implementation in ICartesianSolverImpl.cpp--

    // Get number of joints for which the solver has been configured.
    int getNumJoints() override;

    // Get number of TCPs for which the solver has been configured.
    int getNumTcps() override;

    // Append an additional link.
    bool appendLink(const std::vector<double>& x) override;

    // Restore original kinematic chain.
    bool restoreOriginalChain() override;

    // Change reference frame.
    bool changeOrigin(const std::vector<double> &x_old_obj,
                      const std::vector<double> &x_new_old,
                      std::vector<double> &x_new_obj) override;

    // Perform forward kinematics.
    bool fwdKin(const std::vector<double> &q, std::vector<double> &x) override;

    // Obtain difference between supplied pose inputs.
    bool poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut) override;

    // Perform inverse kinematics.
    bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, const reference_frame frame) override;

    // Perform differential inverse kinematics.
    bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, const reference_frame frame) override;

    // Perform inverse dynamics.
    bool invDyn(const std::vector<double> &q, std::vector<double> &t) override;

    // Perform inverse dynamics.
    bool invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot, const std::vector<double> &ftip, std::vector<double> &t) override;

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    inline const yarp::os::LogComponent & logc() const
    { return !isQuiet ? KDLS() : KDLS_QUIET(); }

    mutable std::mutex mtx;

    /** The chain. **/
    KDL::Chain chain;

    /** To store a copy of the original chain. **/
    KDL::Chain originalChain;

    KDL::ChainFkSolverPos * fkSolverPos {nullptr};
    KDL::ChainIkSolverPos * ikSolverPos {nullptr};
    KDL::ChainIkSolverVel * ikSolverVel {nullptr};
    KDL::ChainIdSolver * idSolver {nullptr};

    bool isQuiet;
};

} // namespace roboticslab

#endif // __KDL_SOLVER_HPP__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_TREE_SOLVER_HPP__
#define __KDL_TREE_SOLVER_HPP__

#include <map>
#include <string>
#include <vector>

#include <yarp/dev/DeviceDriver.h>

#include <kdl/tree.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treeiksolver.hpp>
#include <kdl/treeidsolver.hpp>

#include "ICartesianSolver.h"
#include "KdlTreeSolver_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup KdlTreeSolver
 *
 * @brief Contains KdlTreeSolver.
 */

/**
 * @ingroup KdlTreeSolver
 * @brief The KdlTreeSolver class implements ICartesianSolver.
 */
class KdlTreeSolver : public yarp::dev::DeviceDriver,
                      public roboticslab::ICartesianSolver,
                      public KdlTreeSolver_ParamsParser
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

protected:
    bool makeTree(const yarp::os::Searchable & config);

    std::vector<std::string> endpoints;
    std::map<std::string, std::string> mergedEndpoints;

    KDL::Tree tree;

    KDL::TreeFkSolverPos * fkSolverPos {nullptr};
    KDL::TreeIkSolverPos * ikSolverPos {nullptr};
    KDL::TreeIkSolverVel * ikSolverVel {nullptr};
    KDL::TreeIdSolver * idSolver {nullptr};
};

#endif // __KDL_TREE_SOLVER_HPP__

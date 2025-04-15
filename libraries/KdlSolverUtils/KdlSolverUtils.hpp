// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __KDL_SOLVER_UTILS_HPP__
#define __KDL_SOLVER_UTILS_HPP__

#include <vector>

#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

namespace roboticslab::KdlSolverUtils
{

/**
 * @ingroup kinematics-dynamics-libraries
 * @defgroup KdlSolverUtils
 *
 * @brief Contains utilities related to @ref KdlSolver and @ref KdlTreeSolver.
 */

bool parseKinematicsConfig(const yarp::os::Searchable & config, const std::string & filename, yarp::os::Property & kinematicsConfig);

Eigen::MatrixXd getMatrixFromVector(const std::vector<double> & v);

KDL::JntArray getJntArrayFromVector(const std::vector<double> in);

bool makeChain(const yarp::os::Searchable & kinConfig, KDL::Chain & chain);

} // namespace roboticslab::KdlSolverUtils

#endif // __KDL_SOLVER_UTILS_HPP__

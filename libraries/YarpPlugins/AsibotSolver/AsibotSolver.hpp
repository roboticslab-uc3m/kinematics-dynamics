// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ASIBOT_SOLVER_HPP__
#define __ASIBOT_SOLVER_HPP__

#include <mutex>
#include <vector>

#include <yarp/os/Searchable.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/sig/Matrix.h>

#include "AsibotConfiguration.hpp"
#include "ICartesianSolver.h"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup AsibotSolver
 *
 * @brief Contains roboticslab::AsibotSolver.
 */

/**
 * @ingroup AsibotSolver
 * @brief The AsibotSolver implements ICartesianSolver.
 */
class AsibotSolver : public yarp::dev::DeviceDriver,
                     public ICartesianSolver
{
public:
    // -------- ICartesianSolver declarations. Implementation in ICartesianSolverImpl.cpp --------

    // Get number of joints for which the solver has been configured.
    int getNumJoints() override;

    // Get number of TCPs for which the solver has been configured.
    int getNumTcps() override;

    // Append an additional link.
    bool appendLink(const std::vector<double> &x) override;

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
    bool invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q,
                reference_frame frame) override;

    // Perform differential inverse kinematics.
    bool diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot,
                    reference_frame frame) override;

    // Perform inverse dynamics.
    bool invDyn(const std::vector<double> &q, std::vector<double> &t) override;

    // Perform inverse dynamics.
    bool invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot,
                const std::vector<double> &ftip, std::vector<double> &t, reference_frame frame) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

private:
    struct AsibotTcpFrame
    {
        bool hasFrame;
        yarp::sig::Matrix frameTcp;
    };

    AsibotConfiguration * getConfiguration() const;

    AsibotTcpFrame getTcpFrame() const;
    void setTcpFrame(const AsibotTcpFrame & tcpFrameStruct);

    double A0, A1, A2, A3;  // link lengths

    std::vector<double> qMin, qMax;

    AsibotConfigurationFactory * confFactory {nullptr};

    AsibotTcpFrame tcpFrameStruct;

    mutable std::mutex mtx;
};

} // namespace roboticslab

#endif // __ASIBOT_SOLVER_HPP__

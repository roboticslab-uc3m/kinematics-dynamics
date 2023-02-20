// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __AMOR_CARTESIAN_CONTROL_HPP__
#define __AMOR_CARTESIAN_CONTROL_HPP__

#include <mutex>
#include <vector>

#include <amor.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"
#include "ICartesianSolver.h"

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
class AmorCartesianControl : public yarp::dev::DeviceDriver,
                             public ICartesianControl
{
public:
    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --
    bool stat(std::vector<double> & x, int * state = nullptr, double * timestamp = nullptr) override;
    bool inv(const std::vector<double> & xd, std::vector<double> & q) override;
    bool movj(const std::vector<double> & xd) override;
    bool relj(const std::vector<double> & xd) override;
    bool movl(const std::vector<double> & xd) override;
    bool movv(const std::vector<double> & xdotd) override;
    bool gcmp() override;
    bool forc(const std::vector<double> & fd) override;
    bool stopControl() override;
    bool wait(double timeout) override;
    bool tool(const std::vector<double> & x) override;
    bool act(int command) override;
    void movi(const std::vector<double> & x) override;
    void twist(const std::vector<double> & xdot) override;
    void wrench(const std::vector<double> &w) override;
    bool setParameter(int vocab, double value) override;
    bool getParameter(int vocab, double * value) override;
    bool setParameters(const std::map<int, double> & params) override;
    bool getParameters(std::map<int, double> & params) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    bool checkJointVelocities(const std::vector<double> & qdot);

    AMOR_HANDLE handle {AMOR_INVALID_HANDLE};
    bool ownsHandle {true};
    mutable std::mutex * handleMutex {nullptr};

    yarp::dev::PolyDriver cartesianDevice;
    ICartesianSolver * iCartesianSolver;

    int currentState;
    double gain;
    int waitPeriodMs;

    std::vector<double> qdotMax;

    ICartesianSolver::reference_frame referenceFrame;
};

} // namespace roboticslab

#endif // __AMOR_CARTESIAN_CONTROL_HPP__
